#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visual_based_planning/PlanVisibilityPath.h>
#include <random>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>

// Structure to hold a pre-generated target problem
struct TargetProblem {
    int id;
    float cx, cy, cz;
    float radius;
    std::vector<float> points_flat; // The actual flat vector for the ROS param
};

// Configuration structure to define an experiment
struct ExperimentConfig {
    std::string name;
    std::string planner_mode;        // "VisRRT" or "VisPRM"
    bool use_visual_ik;
    bool use_visibility_integrity;
};

// Helper struct to store results
struct ExperimentResult {
    std::string config_name;
    double first_run_time;                 // Initialization/Cold start time
    std::vector<double> subsequent_times;  // Remaining 99 runs
    
    // Success tracking
    int success_count = 0;
    int fail_count = 0;
    std::vector<int> outcomes;             // 1 = Success, 0 = Fail (for all 100 runs)

    // Runtime Stats
    double avg, median, min, max, std_dev;
};

class ExperimentRunner {
public:
    ExperimentRunner(ros::NodeHandle& nh) : nh_(nh) {
        // Setup Service Client
        client_ = nh_.serviceClient<visual_based_planning::PlanVisibilityPath>("plan_visibility_path");
        
        // Random number generators
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    void runExperiments() {
        ROS_INFO("Waiting for planning service...");
        client_.waitForExistence();
        ROS_INFO("Service available.");

        // 1. Pre-compute the 100 problems so all configs face the exact same challenges
        ROS_INFO("Pre-generating 100 target problems...");
        generateTargetProblems();

        // Define the 6 Configurations
        std::vector<ExperimentConfig> configs = {
            // RRT Configurations
            {"RRT_VisualIK_ON",  "VisRRT", true,  false},
            {"RRT_VisualIK_OFF", "VisRRT", false, false},

            // PRM Configurations
            {"PRM_VisualIK_ON_Integrity_ON",   "VisPRM", true,  true},
            {"PRM_VisualIK_ON_Integrity_OFF",  "VisPRM", true,  false},
            {"PRM_VisualIK_OFF_Integrity_ON",  "VisPRM", false, true},
            {"PRM_VisualIK_OFF_Integrity_OFF", "VisPRM", false, false}
        };

        std::vector<ExperimentResult> all_results;

        for (const auto& cfg : configs) {
            ROS_INFO("------------------------------------------------");
            ROS_INFO("Starting Configuration: %s", cfg.name.c_str());
            
            // 2. Set Parameters for this configuration
            nh_.setParam("/planner/mode", cfg.planner_mode);
            nh_.setParam("/planner/use_visual_ik", cfg.use_visual_ik);
            nh_.setParam("/planner/visibility_integrity/enabled", cfg.use_visibility_integrity);
            nh_.setParam("/planner/use_visibility_integrity", cfg.use_visibility_integrity);

            ros::Duration(0.5).sleep(); // Ensure params propagate

            ExperimentResult result;
            result.config_name = cfg.name;

            // 3. Run the 100 Pre-computed Trials
            int num_trials = 1;
            for (int i = 0; i < num_trials; ++i) {
                
                // Upload the i-th problem to the ROS param server
                nh_.setParam("/target_points", problems_[i].points_flat);

                // Prepare Service Request
                visual_based_planning::PlanVisibilityPath srv;
                const std::vector<float>& pts = problems_[i].points_flat;
                for (size_t k = 0; k + 2 < pts.size(); k += 3) {
                    geometry_msgs::Point p;
                    p.x = pts[k];
                    p.y = pts[k+1];
                    p.z = pts[k+2];
                    srv.request.task.target_points.push_back(p);
                }

                // Call Service and Measure Time
                ros::WallTime start_time = ros::WallTime::now();
                bool call_success = client_.call(srv);
                ros::WallTime end_time = ros::WallTime::now();
                
                double duration = (end_time - start_time).toSec();

                // Determine Success/Fail
                bool run_success = (call_success && srv.response.success);

                if (run_success) {
                    result.success_count++;
                    result.outcomes.push_back(1);
                } else {
                    result.fail_count++;
                    result.outcomes.push_back(0);
                    if (!call_success) ROS_WARN("Service call failed (Comm Error) on trial %d", i);
                }

                // Store Times
                if (i == 0) {
                    result.first_run_time = duration;
                    ROS_INFO("  [Init]: %.4f sec | Result: %s", duration, run_success ? "SUCCESS" : "FAIL");
                } else {
                    result.subsequent_times.push_back(duration);
                }

                if (i % 10 == 0 && i > 0) ROS_INFO("  Progress: %d/%d", i, num_trials);
            }

            computeStats(result);
            printStats(result);
            all_results.push_back(result);
        }

        saveToFile(all_results);
        ROS_INFO("Experiments completed. Data saved to 'planner_experiments_results.txt'.");
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient client_;
    std::mt19937 rng_;
    std::vector<TargetProblem> problems_; // Storage for the fixed set of 100 problems

    void generateTargetProblems() {
        problems_.clear();
        problems_.reserve(100);

        // Distributions setup
        std::uniform_real_distribution<float> dist_z(0.2, 2.5);
        std::uniform_real_distribution<float> dist_outer(-2.5, 2.5);
        std::uniform_real_distribution<float> dist_r(0.03, 0.3);
        std::normal_distribution<float> dist_norm(0.0, 1.0);
        std::uniform_real_distribution<float> dist_u(0.0, 1.0);

        for (int i = 0; i < 100; ++i) {
            TargetProblem prob;
            prob.id = i;

            // 1. Center Generation (Rectangular Annulus)
            prob.cz = dist_z(rng_);
            
            while (true) {
                prob.cx = dist_outer(rng_);
                prob.cy = dist_outer(rng_);
                
                // Reject if inside [-1.5, 1.5]^2
                if (std::abs(prob.cx) < 1.5 && std::abs(prob.cy) < 1.5) {
                    continue;
                }
                break;
            }

            // 2. Radius
            prob.radius = dist_r(rng_);

            // 3. Points Generation
            prob.points_flat.reserve(100 * 3);
            for (int j = 0; j < 100; ++j) {
                float x = dist_norm(rng_);
                float y = dist_norm(rng_);
                float z = dist_norm(rng_);
                
                float norm = std::sqrt(x*x + y*y + z*z);
                x /= norm; y /= norm; z /= norm;

                float r_scale = prob.radius * std::cbrt(dist_u(rng_));

                prob.points_flat.push_back(prob.cx + x * r_scale);
                prob.points_flat.push_back(prob.cy + y * r_scale);
                prob.points_flat.push_back(prob.cz + z * r_scale);
            }

            problems_.push_back(prob);
        }
    }

    void computeStats(ExperimentResult& res) {
        if (res.subsequent_times.empty()) return;
        
        double sum = std::accumulate(res.subsequent_times.begin(), res.subsequent_times.end(), 0.0);
        res.avg = sum / res.subsequent_times.size();

        std::vector<double> sorted = res.subsequent_times;
        std::sort(sorted.begin(), sorted.end());
        res.min = sorted.front();
        res.max = sorted.back();
        
        if (sorted.size() % 2 == 0)
            res.median = (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]) / 2.0;
        else
            res.median = sorted[sorted.size()/2];

        double sq_sum = std::inner_product(res.subsequent_times.begin(), res.subsequent_times.end(), res.subsequent_times.begin(), 0.0);
        double variance = (sq_sum / res.subsequent_times.size()) - (res.avg * res.avg);
        res.std_dev = std::sqrt(variance);
    }

    void printStats(const ExperimentResult& res) {
        std::cout << "\nResults for " << res.config_name << ":\n";
        std::cout << "  Successes: " << res.success_count << " | Failures: " << res.fail_count << "\n";
        std::cout << "  Init Time: " << res.first_run_time << " s\n";
        std::cout << "  Avg (Warm):" << res.avg << " s\n";
        std::cout << "  Median:    " << res.median << " s\n";
        std::cout << "  Min:       " << res.min << " s\n";
        std::cout << "  Max:       " << res.max << " s\n";
        std::cout << "  Std Dev:   " << res.std_dev << " s\n\n";
    }

    void saveToFile(const std::vector<ExperimentResult>& results) {
        std::ofstream file("planner_experiments_results.txt");
        if (!file.is_open()) {
            ROS_ERROR("Unable to open file for writing results.");
            return;
        }

        file << "Experiment Results\n";
        file << "==================\n";

        for (const auto& res : results) {
            file << "Configuration: " << res.config_name << "\n";
            file << "Total Successes: " << res.success_count << "\n";
            file << "Total Failures:  " << res.fail_count << "\n";
            file << "Initialization Time: " << res.first_run_time << "\n";
            
            file << "Runtime Stats (Subsequent 99 Runs):\n";
            file << "  Average: " << res.avg << "\n";
            file << "  Median:  " << res.median << "\n";
            file << "  Min:     " << res.min << "\n";
            file << "  Max:     " << res.max << "\n";
            file << "  StdDev:  " << res.std_dev << "\n";
            
            file << "Run Outcomes (1=Success, 0=Fail): ";
            for (size_t i = 0; i < res.outcomes.size(); ++i) {
                file << res.outcomes[i] << (i < res.outcomes.size() - 1 ? ", " : "");
            }
            file << "\n";

            file << "Raw Times (Run 0 to 99): " << res.first_run_time;
            for (double t : res.subsequent_times) {
                file << ", " << t;
            }
            file << "\n\n";
        }
        file.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "experiment_runner");
    ros::NodeHandle nh;
    ExperimentRunner runner(nh);
    runner.runExperiments();
    return 0;
}