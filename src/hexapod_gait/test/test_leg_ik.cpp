#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "hexapod_gait/hexapod_ik.hpp"
#include "gait_strategy/gait_strategy.hpp"
#include "gait_factory/gait_factory.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using hexapod_gait::LEG;
using hexapod_gait::LegData;

struct LegAngleTestData {
    int frame;
    std::string leg_name;
    double expected_coxa;
    double expected_femur;
    double expected_tibia;
    double mount_angle;
    double step_offset;
    double base_x; // in mm
    double base_y; // in mm
    double base_z; // in mm
};

std::vector<LegAngleTestData> loadAngleCSV(const std::string &filepath) {
    std::vector<LegAngleTestData> data;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        throw std::runtime_error("Failed to open CSV file: " + filepath);
    }

    std::string line;
    // Skip header line
    if (!std::getline(file, line)) {
        return data;
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string item;
        LegAngleTestData row;

        std::getline(ss, item, ','); row.frame = std::stoi(item);
        std::getline(ss, row.leg_name, ',');
        std::getline(ss, item, ','); row.expected_coxa = std::stod(item);
        std::getline(ss, item, ','); row.expected_femur = std::stod(item);
        std::getline(ss, item, ','); row.expected_tibia = std::stod(item);
        std::getline(ss, item, ','); row.mount_angle = std::stod(item);
        std::getline(ss, item, ','); row.step_offset = std::stod(item);
        std::getline(ss, item, ','); row.base_x = std::stod(item);
        std::getline(ss, item, ','); row.base_y = std::stod(item);
        std::getline(ss, item, ','); row.base_z = std::stod(item);

        data.push_back(row);
    }
    return data;
}

TEST(HexapodIkTest, ValidateLegPropagation) {
    std::string package_share_dir;
    ASSERT_NO_THROW({
        package_share_dir = ament_index_cpp::get_package_share_directory("hexapod_gait");
    }) << "Failed to locate hexapod_gait package share directory.";

    std::string csv_path = package_share_dir + "/test/angle_history.csv";

    std::vector<LegAngleTestData> dataset;
    ASSERT_NO_THROW({
        dataset = loadAngleCSV(csv_path);
    }) << "Failed to load CSV test dataset from: " << csv_path;

    ASSERT_FALSE(dataset.empty()) << "CSV dataset is empty!";

    auto gait_strategy = hexapod_gait::GaitFactory::create_gait("tripod_gait", 30);
    ASSERT_NE(gait_strategy, nullptr) << "Failed to create tripod_gait strategy!";

    const double STRIDE_LENGTH = 0.080;
    const double SWING_HEIGHT = 0.040;
    const double GLOBAL_DIRECTION = M_PI;
    Eigen::Vector3d relative_target(0.10, 0.0, -0.08);

    // Tolerance in radians (~0.57 degrees tolerance: 0.01 rad)
    constexpr double TOLERANCE = 10e-3;

    static const std::unordered_map<std::string, LEG> leg_name_map = {
        {"LF_LEG", hexapod_gait::LF_LEG},
        {"LM_LEG", hexapod_gait::LM_LEG},
        {"LB_LEG", hexapod_gait::LR_LEG},
        {"LR_LEG", hexapod_gait::LR_LEG},
        {"RF_LEG", hexapod_gait::RF_LEG},
        {"RM_LEG", hexapod_gait::RM_LEG},
        {"RB_LEG", hexapod_gait::RR_LEG},
        {"RR_LEG", hexapod_gait::RR_LEG}
    };

    for (const auto &row : dataset) {
        SCOPED_TRACE("Frame " + std::to_string(row.frame) + " | Leg: " + row.leg_name);

        std::unordered_map<LEG, LegData> leg_data = gait_strategy->propagate_gait(
            row.frame, STRIDE_LENGTH, SWING_HEIGHT, relative_target, GLOBAL_DIRECTION);

        LEG leg_enum = leg_name_map.at(row.leg_name);
        const LegData &calculated_leg = leg_data.at(leg_enum);

        std::cout << "Frame " << row.frame << " | Leg " << row.leg_name 
                    << " | Coxa Calc: " << calculated_leg.coxa_joint_ << " Exp: " << row.expected_coxa
                    << " | Femur Calc: " << calculated_leg.femur_joint_ << " Exp: " << row.expected_femur
                    << " | Tibia Calc: " << calculated_leg.tibia_joint_ << " Exp: " << row.expected_tibia << std::endl;

        EXPECT_NEAR(calculated_leg.coxa_joint_, row.expected_coxa, TOLERANCE)  << "Coxa joint angle mismatch";
        EXPECT_NEAR(calculated_leg.femur_joint_, row.expected_femur, TOLERANCE) << "Femur joint angle mismatch";
        EXPECT_NEAR(calculated_leg.tibia_joint_, row.expected_tibia, TOLERANCE) << "Tibia joint angle mismatch";
    }
}


TEST(HexapodIkTest, ValidateLegOffset) {
    std::string package_share_dir;
    ASSERT_NO_THROW({
        package_share_dir = ament_index_cpp::get_package_share_directory("hexapod_gait");
    }) << "Failed to locate hexapod_gait package share directory.";

    std::string csv_path = package_share_dir + "/test/angle_history.csv";

    std::vector<LegAngleTestData> dataset;
    ASSERT_NO_THROW({
        dataset = loadAngleCSV(csv_path);
    }) << "Failed to load CSV test dataset from: " << csv_path;

    ASSERT_FALSE(dataset.empty()) << "CSV dataset is empty!";

    auto gait_strategy = hexapod_gait::GaitFactory::create_gait("tripod_gait", 30);
    ASSERT_NE(gait_strategy, nullptr) << "Failed to create tripod_gait strategy!";

    const double STRIDE_LENGTH = 0.080;
    const double SWING_HEIGHT = 0.040;
    const double GLOBAL_DIRECTION = M_PI;
    Eigen::Vector3d relative_target(0.10, 0.0, -0.08);

    // Tolerance in radians (~0.57 degrees tolerance: 0.01 rad)
    constexpr double TOLERANCE = 10e-3;

    static const std::unordered_map<std::string, LEG> leg_name_map = {
        {"LF_LEG", hexapod_gait::LF_LEG},
        {"LM_LEG", hexapod_gait::LM_LEG},
        {"LB_LEG", hexapod_gait::LR_LEG},
        {"LR_LEG", hexapod_gait::LR_LEG},
        {"RF_LEG", hexapod_gait::RF_LEG},
        {"RM_LEG", hexapod_gait::RM_LEG},
        {"RB_LEG", hexapod_gait::RR_LEG},
        {"RR_LEG", hexapod_gait::RR_LEG}
    };

    for (const auto &row : dataset) {
        SCOPED_TRACE("Frame " + std::to_string(row.frame) + " | Leg: " + row.leg_name);

        std::unordered_map<LEG, LegData> leg_data = gait_strategy->propagate_gait(
            row.frame, STRIDE_LENGTH, SWING_HEIGHT, relative_target, GLOBAL_DIRECTION);

        LEG leg_enum = leg_name_map.at(row.leg_name);
        const LegData &calculated_leg = leg_data.at(leg_enum);

        std::cout << "Frame " << row.frame << " | Leg " << row.leg_name 
                    << " | Coxa Calc: " << calculated_leg.coxa_joint_ << " Exp: " << row.expected_coxa
                    << " | Femur Calc: " << calculated_leg.femur_joint_ << " Exp: " << row.expected_femur
                    << " | Tibia Calc: " << calculated_leg.tibia_joint_ << " Exp: " << row.expected_tibia << std::endl;

        EXPECT_NEAR(calculated_leg.coxa_joint_, row.expected_coxa, TOLERANCE)  << "Coxa joint angle mismatch";
        EXPECT_NEAR(calculated_leg.femur_joint_, row.expected_femur, TOLERANCE) << "Femur joint angle mismatch";
        EXPECT_NEAR(calculated_leg.tibia_joint_, row.expected_tibia, TOLERANCE) << "Tibia joint angle mismatch";
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
