#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include <memory>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "ground_truth_package.h"
#include "LaserMeasurement.h"
#include "RadarMeasurement.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void check_arguments
(
  int argc, 
  char *argv[]
) 
{
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1)
    cerr << usage_instructions << endl;
  else if (argc == 2)
    cerr << "Please include an output file.\n" << usage_instructions << endl;
  else if (argc == 3)
    has_valid_args = true;
  else if (argc > 3)
    cerr << "Too many arguments.\n" << usage_instructions << endl;

  if (!has_valid_args)  exit(EXIT_FAILURE);
}

void check_files
(
  ifstream &in_file, 
  string &in_name,
  ofstream &out_file, 
  string &out_name
) 
{
  if (!in_file.is_open()) 
  {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open()) 
  {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main
(
  int argc, 
  char *argv[]
) 
{

  check_arguments(argc, argv);

  string in_file_name_ = argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  // Measurement vector
  vector<shared_ptr<MeasurementPackage>> measurement_pack_list;
  // Ground truth vector
  vector<GroundTruthPackage> gt_pack_list;

  string line;

  // Read from the data
  while (getline(in_file_, line))
  {
    string sensor_type;
    GroundTruthPackage gt_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;
    if (!sensor_type.compare("L")) 
    {
      // LASER MEASUREMENT
      LaserMeasurement meas_package;

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      double x, y;
      iss >> x;
      iss >> y;
      meas_package.raw_measurements_ << x, y;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(make_shared<LaserMeasurement>(meas_package));
    } 
    else if (!sensor_type.compare("R")) 
    {
      // RADAR MEASUREMENT
      RadarMeasurement meas_package;

      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      double rho, phi, rho_dot;
      iss >> rho;
      iss >> phi;
      iss >> rho_dot;
      meas_package.raw_measurements_ << rho, phi, rho_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(make_shared<RadarMeasurement>(meas_package));
    }

    // read ground truth data to compare later
    double x_gt, y_gt, vx_gt, vy_gt;
    iss >> x_gt;
    iss >> y_gt;
    iss >> vx_gt;
    iss >> vy_gt;
    gt_package.gt_values_ = VectorXd(4);
    gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
    gt_pack_list.push_back(gt_package);
  }

  // Create a Fusion EKF instance
  FusionEKF fusionEKF(4);

  // used to compute the RMSE later
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  //Call the EKF-based fusion
  size_t N = measurement_pack_list.size();

  for (size_t k = 0; k < N; ++k) 
  {
    // Get the current measurement package
    shared_ptr<MeasurementPackage> meas = measurement_pack_list[k];

    if (fusionEKF.ProcessMeasurement(*meas)) 
    {
      // output the estimation
      out_file_ << fusionEKF.ekf_.x_(0) << "\t";
      out_file_ << fusionEKF.ekf_.x_(1) << "\t";
      out_file_ << fusionEKF.ekf_.x_(2) << "\t";
      out_file_ << fusionEKF.ekf_.x_(3) << "\t";

      // output the measurements
      if (meas->sensor_type_ == MeasurementPackage::LASER) 
      {
        // output the estimation
        out_file_ << meas->raw_measurements_(0) << "\t";
        out_file_ << meas->raw_measurements_(1) << "\t";
      } 
      else if (meas->sensor_type_ == MeasurementPackage::RADAR) 
      {
        // output the estimation in the cartesian coordinates
        const double rho = meas->raw_measurements_(0);
        const double phi = meas->raw_measurements_(1);
        out_file_ << rho * cos(phi) << "\t";
        out_file_ << rho * sin(phi) << "\t";
      }

      // output the ground truth packages
      out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
      out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
      out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
      out_file_ << gt_pack_list[k].gt_values_(3) << "\n";

      estimations.push_back(fusionEKF.ekf_.x_);
      ground_truth.push_back(gt_pack_list[k].gt_values_);
    }
  }

  // compute the accuracy (RMSE)
  cout << "Accuracy - RMSE:" << endl;
  cout << Tools().CalculateRMSE(estimations, ground_truth) << endl;

  // close files
  if (out_file_.is_open())  out_file_.close();
  if (in_file_.is_open())   in_file_.close();

  return 0;
}
