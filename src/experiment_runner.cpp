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
    std::vector<float> points_flat; 
};

struct ExperimentConfig {
    std::string name;
    std::string planner_mode;        
    bool use_visual_ik;
    bool use_visibility_integrity;
};

struct Stats {
    double avg = 0.0;
    double median = 0.0;
    double min = 0.0;
    double max = 0.0;
    double std_dev = 0.0;
    int count = 0;
};

struct ExperimentResult {
    std::string config_name;
    double first_run_time;                 
    
    // Raw Data (Runs 1-99, excluding init)
    std::vector<double> all_times; 
    std::vector<double> successful_times;

    // Success tracking
    int success_count = 0;
    int fail_count = 0;
    std::vector<int> outcomes;             

    // Computed Stats
    Stats stats_all;
    Stats stats_success;
};

class ExperimentRunner {
public:
    ExperimentRunner(ros::NodeHandle& nh) : nh_(nh) {
        client_ = nh_.serviceClient<visual_based_planning::PlanVisibilityPath>("plan_visibility_path");
        std::random_device rd;
        rng_ = std::mt19937(rd());
    }

    void runExperiments() {
        ROS_INFO("Waiting for planning service...");
        client_.waitForExistence();
        ROS_INFO("Service available.");

        ROS_INFO("Pre-generating 100 target problems...");
        generateTargetProblems();

        std::vector<ExperimentConfig> configs = {
            // {"RRT_VisualIK_ON",  "VisRRT", true,  false},
            // {"RRT_VisualIK_OFF", "VisRRT", false, false},
            // {"PRM_VisualIK_ON_Integrity_ON",   "VisPRM", true,  true},
            // {"PRM_VisualIK_ON_Integrity_OFF",  "VisPRM", true,  false},
            {"PRM_VisualIK_OFF_Integrity_ON",  "VisPRM", false, true},
            {"PRM_VisualIK_OFF_Integrity_OFF", "VisPRM", false, false}
        };

        std::vector<ExperimentResult> all_results;

        for (const auto& cfg : configs) {
            ROS_INFO("------------------------------------------------");
            ROS_INFO("Starting Configuration: %s", cfg.name.c_str());
            
            nh_.setParam("/planner/mode", cfg.planner_mode);
            nh_.setParam("/planner/use_visual_ik", cfg.use_visual_ik);
            nh_.setParam("/planner/visibility_integrity/enabled", cfg.use_visibility_integrity);
            nh_.setParam("/planner/use_visibility_integrity", cfg.use_visibility_integrity);

            ros::Duration(0.5).sleep(); 

            ExperimentResult result;
            result.config_name = cfg.name;

            int num_trials = 100;
            for (int i = 0; i < num_trials; ++i) {
                
                nh_.setParam("/target_points", problems_[i].points_flat);

                visual_based_planning::PlanVisibilityPath srv;
                const std::vector<float>& pts = problems_[i].points_flat;
                for (size_t k = 0; k + 2 < pts.size(); k += 3) {
                    geometry_msgs::Point p;
                    p.x = pts[k];
                    p.y = pts[k+1];
                    p.z = pts[k+2];
                    srv.request.task.target_points.push_back(p);
                }

                ros::WallTime start_time = ros::WallTime::now();
                bool call_success = client_.call(srv);
                ros::WallTime end_time = ros::WallTime::now();
                
                double duration = (end_time - start_time).toSec();
                bool run_success = (call_success && srv.response.success);

                // Track Outcomes
                if (run_success) {
                    result.success_count++;
                    result.outcomes.push_back(1);
                } else {
                    result.fail_count++;
                    result.outcomes.push_back(0);
                    if (!call_success) ROS_WARN("Service call failed (Comm Error) on trial %d", i);
                }

                // Store Data
                if (i == 0) {
                    result.first_run_time = duration;
                    ROS_INFO("  [Init]: %.4f sec | Result: %s", duration, run_success ? "SUCCESS" : "FAIL");
                }
                result.all_times.push_back(duration);
                if (run_success) {
                    result.successful_times.push_back(duration);
                }
 
                if (i % 10 == 0 && i > 0) ROS_INFO("  Progress: %d/%d", i, num_trials);
            }

            // Compute Stats for both All and Successful sets
            result.stats_all = computeStats(result.all_times);
            result.stats_success = computeStats(result.successful_times);
            
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
    std::vector<TargetProblem> problems_;

    void generateTargetProblems() {
        problems_.clear();
        problems_.reserve(100);

        std::uniform_real_distribution<float> dist_z(0.2, 2.0);
        std::uniform_real_distribution<float> dist_outer(-2.5, 2.5);
        std::uniform_real_distribution<float> dist_r(0.03, 0.5);
        std::normal_distribution<float> dist_norm(0.0, 1.0);
        std::uniform_real_distribution<float> dist_u(0.0, 1.0);

        for (int i = 0; i < 100; ++i) {
            TargetProblem prob;
            prob.id = i;
            prob.cz = dist_z(rng_);
            
            while (true) {
                prob.cx = dist_outer(rng_);
                prob.cy = dist_outer(rng_);
                if (std::abs(prob.cx) < 1.5 && std::abs(prob.cy) < 1.5) continue;
                break;
            }

            prob.radius = dist_r(rng_);
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

    Stats computeStats(const std::vector<double>& data) {
        Stats s;
        s.count = data.size();
        if (data.empty()) return s;
        
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        s.avg = sum / data.size();

        std::vector<double> sorted = data;
        std::sort(sorted.begin(), sorted.end());
        s.min = sorted.front();
        s.max = sorted.back();
        
        if (sorted.size() % 2 == 0)
            s.median = (sorted[sorted.size()/2 - 1] + sorted[sorted.size()/2]) / 2.0;
        else
            s.median = sorted[sorted.size()/2];

        double sq_sum = std::inner_product(data.begin(), data.end(), data.begin(), 0.0);
        double variance = (sq_sum / data.size()) - (s.avg * s.avg);
        s.std_dev = std::sqrt(std::max(0.0, variance)); // max with 0 to avoid tiny negative from float precision
        return s;
    }

    void printStats(const ExperimentResult& res) {
        std::cout << "\nResults for " << res.config_name << ":\n";
        std::cout << "  Success: " << res.success_count << " | Fail: " << res.fail_count << "\n";
        std::cout << "  Init Time: " << res.first_run_time << " s\n";
        
        std::cout << "  -- All Runs --\n";
        std::cout << "    Avg: " << res.stats_all.avg << ", Med: " << res.stats_all.median << "\n";
        
        std::cout << "  -- Successful Runs Only --\n";
        if (res.stats_success.count > 0) {
            std::cout << "    Avg: " << res.stats_success.avg << ", Med: " << res.stats_success.median << "\n";
            std::cout << "    Min: " << res.stats_success.min << ", Max: " << res.stats_success.max << "\n";
        } else {
            std::cout << "    (No successful runs to analyze)\n";
        }
        std::cout << "\n";
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
            
            file << "\n--- Stats: ALL Runs ---\n";
            file << "  Count:   " << res.stats_all.count << "\n";
            file << "  Average: " << res.stats_all.avg << "\n";
            file << "  Median:  " << res.stats_all.median << "\n";
            file << "  Min:     " << res.stats_all.min << "\n";
            file << "  Max:     " << res.stats_all.max << "\n";
            file << "  StdDev:  " << res.stats_all.std_dev << "\n";

            file << "\n--- Stats: SUCCESSFUL Runs Only ---\n";
            file << "  Count:   " << res.stats_success.count << "\n";
            if (res.stats_success.count > 0) {
                file << "  Average: " << res.stats_success.avg << "\n";
                file << "  Median:  " << res.stats_success.median << "\n";
                file << "  Min:     " << res.stats_success.min << "\n";
                file << "  Max:     " << res.stats_success.max << "\n";
                file << "  StdDev:  " << res.stats_success.std_dev << "\n";
            } else {
                file << "  (N/A - No Successes)\n";
            }
            
            file << "\nRun Outcomes (1=Success, 0=Fail): ";
            for (size_t i = 0; i < res.outcomes.size(); ++i) {
                file << res.outcomes[i] << (i < res.outcomes.size() - 1 ? ", " : "");
            }
            file << "\n";

            file << "Raw Times (Run 0 to 99): ";
            for (double t : res.all_times) {
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