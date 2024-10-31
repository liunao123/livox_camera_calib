#include "include/lidar_camera_calib.hpp"

#include "ceres/ceres.h"
#include "include/common.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/core/eigen.hpp>

using namespace std;

// Data path
string image_file;
string pcd_file;
string result_file;

// Camera config
vector<double> camera_matrix;
vector<double> dist_coeffs;
double width;
double height;

// Calib config
bool use_rough_calib;
string calib_config_file;
// instrins matrix
Eigen::Matrix3d inner;
// Distortion coefficient
Eigen::Vector4d distor;
Eigen::Vector4d quaternion;
Eigen::Vector3d transation;

Eigen::Matrix3d R;
Eigen::Vector3d T;


// Normal pnp solution
class pnp_calib
{
public:
  pnp_calib(PnPData p) { pd = p; }
  template <typename T>
  bool operator()(const T *_q, const T *_t, T *residuals) const
  {
    Eigen::Matrix<T, 3, 3> innerT = inner.cast<T>();
    Eigen::Matrix<T, 4, 1> distorT = distor.cast<T>();
    Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
    q_incre.normalize();
    Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};
    Eigen::Matrix<T, 3, 1> p_l(T(pd.x), T(pd.y), T(pd.z));
    Eigen::Matrix<T, 3, 1> p_c = q_incre.toRotationMatrix() * p_l + t_incre;
    Eigen::Matrix<T, 3, 1> p_2 = innerT * p_c;

    // 深度归一化
    T uo = p_2[0] / p_2[2];
    T vo = p_2[1] / p_2[2];
    // 相机内参
    const T &fx = innerT.coeffRef(0, 0);
    const T &cx = innerT.coeffRef(0, 2);
    const T &fy = innerT.coeffRef(1, 1);
    const T &cy = innerT.coeffRef(1, 2);

    T xo = (uo - cx) / fx;
    T yo = (vo - cy) / fy;
    T r2 = xo * xo + yo * yo;
    T r4 = r2 * r2;
    // 畸变参数
    T distortion = 1.0 + distorT[0] * r2 + distorT[1] * r4;
    T xd = xo * distortion + (distorT[2] * xo * yo + distorT[2] * xo * yo) +
           distorT[3] * (r2 + xo * xo + xo * xo);
    T yd = yo * distortion + distorT[3] * xo * yo + distorT[3] * xo * yo +
           distorT[2] * (r2 + yo * yo + yo * yo);
    // std::cout <<  "54:  " << xo << " " << yo << " " << xd << " " << yd << std::endl;
    T ud = fx * xd + cx;
    T vd = fy * yd + cy;
    // residuals[0] = ud - T(pd.u);
    // residuals[1] = vd - T(pd.v);

    T error_u = ceres::abs( ud - T(pd.u) ) ;
    T error_v = ceres::abs( vd - T(pd.v) ) ;
    residuals[0] = error_u * error_u + error_v * error_v ;

    // std::cout <<  "69:  " << ud << " " << vd << " " << T(pd.u) << " " << T(pd.v) << std::endl;
 
  }
  static ceres::CostFunction *Create(PnPData p)
  {
    return (
        // new ceres::AutoDiffCostFunction<pnp_calib, 2, 4, 3>(new pnp_calib(p)));
    new ceres::AutoDiffCostFunction<pnp_calib, 1, 4, 3>(new pnp_calib(p)));
  }

private:
  PnPData pd;
};

void solver_2d_3d(  std::vector<PnPData> pnp_list  )
{
  // 初始旋转矩阵
  Eigen::Matrix3d init_rotation_matrix_;
  // init_rotation_matrix_.setIdentity();

  // init_rotation_matrix_ << -0., -1.0, -0.,
  //     0., 0., -1.0,
  //     1.0, -0., 0.;

  // 上次的结果作为初值
  init_rotation_matrix_ = R;
  // 初始平移向量
  Eigen::Vector3d init_translation_vector_ = T;

  // init_translation_vector_ << 0.0, 0.08, -0.03;

  std::cout << "init_rotation_matrix_:" << init_rotation_matrix_ << std::endl;
  std::cout << "init_translation_vector_:" << init_translation_vector_ << std::endl;

  Eigen::Quaterniond q(init_rotation_matrix_);
  q.normalize();
  Eigen::Vector3d ori_t = init_translation_vector_;
  double ext[7];
  ext[0] = q.x();
  ext[1] = q.y();
  ext[2] = q.z();
  ext[3] = q.w();
  ext[4] = init_translation_vector_[0];
  ext[5] = init_translation_vector_[1];
  ext[6] = init_translation_vector_[2];
  Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(ext);
  Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(ext + 4);

  ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
  ceres::Problem problem;

  problem.AddParameterBlock(ext, 4, q_parameterization);
  problem.AddParameterBlock(ext + 4, 3);

  // ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0); // 使用 Huber Loss，并设置参数
  cout << "137----------------------------------------  pnp_list size: " << pnp_list.size() << std::endl << std::endl;

  for (auto val : pnp_list)
  {
    ceres::CostFunction *cost_function;
    cost_function = pnp_calib::Create(val);
    problem.AddResidualBlock(cost_function, NULL, ext, ext + 4);
    // problem.AddResidualBlock(cost_function, loss_function, ext, ext + 4);
  }

  // problem.SetParameterLowerBound(ext + 4, 0, -0.02);
  // problem.SetParameterUpperBound(ext + 4, 0,  0.02);

  // problem.SetParameterLowerBound(ext + 4, 1, -0.010);
  // problem.SetParameterUpperBound(ext + 4, 1,  0.01);

  // problem.SetParameterLowerBound(ext + 4, 2,  -0.15);
  // problem.SetParameterUpperBound(ext + 4, 2,  0.15);

  ceres::Solver::Options options;
  options.preconditioner_type = ceres::JACOBI;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 200;  // 设置最大迭代次数为 100
  options.function_tolerance = 1e-6; // 设置函数值的收敛阈值为 1e-6


  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << std::endl;

  Eigen::Matrix3d rot = m_q.toRotationMatrix();
  R = rot;
  // Eigen::Vector3d T;
  T[0] = m_t(0);
  T[1] = m_t(1);
  T[2] = m_t(2);

  for (size_t i = 0; i < 7; i++)
  {
    cout << " " << ext[i];
  }
  cout << endl << endl;
  cout << "result_file: " << result_file  << endl;

  static std::ofstream outfile11(result_file);
  std::ofstream outfile(result_file, ios_base::app);

  outfile << std::endl
          << "T_cl: " << std::endl;

  for (int i = 0; i < 3; i++)
  {
    outfile << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "," << T[i] << std::endl;
    std::cout << R(i, 0) << "," << R(i, 1) << "," << R(i, 2) << "," << T[i] << std::endl;
  }

  outfile << 0 << "," << 0 << "," << 0 << "," << 1 << std::endl;

  outfile << std::endl
          << "T_lc: " << std::endl;
  Eigen::Isometry3d T_cl = Eigen::Isometry3d::Identity();
  T_cl.rotate(R);
  T_cl.pretranslate(T);

  outfile << std::endl
          << "T_lc: " << std::endl
          << T_cl.inverse().matrix() << std::endl;

  outfile << std::endl
          << "R :<ZYX> " << std::endl;

  Eigen::Vector3d R_euler = R.eulerAngles(2, 1, 0);

  outfile << RAD2DEG(R_euler[0]) << "," << RAD2DEG(R_euler[1]) << "," << RAD2DEG(R_euler[2])
          << std::endl
          << std::endl
          << std::endl;
  cout << "result_path: " << result_file << std::endl
       << std::endl;

  Eigen::Matrix3d adjust_rotation = init_rotation_matrix_.inverse() * R;
  Eigen::Vector3d adjust_euler = adjust_rotation.eulerAngles(2, 1, 0);
  outfile << RAD2DEG(adjust_euler[0]) << "," << RAD2DEG(adjust_euler[1]) << "," << RAD2DEG(adjust_euler[2]) << std::endl
          << std::endl;

}

std::vector<PnPData> compute_residuals_error( const  std::vector<PnPData> pnp_list, 
                            const cv::Mat cv_camera_matrix,
                            const cv::Mat cv_distortion_coeff )
{
  cv::Mat cv_r_vec =
      (cv::Mat_<double>(3, 3) << R(0, 0), R(0, 1), R(0, 2),
       R(1, 0), R(1, 1), R(1, 2),
       R(2, 0), R(2, 1), R(2, 2));

  cv::Mat cv_t_vec = (cv::Mat_<double>(3, 1) << T(0), T(1), T(2));

  std::vector<cv::Point3f> pts_3d;
  std::vector<cv::Point2f> pts_2d;
  for (auto val : pnp_list)
  {
    cv::Point3f p3;
    p3.x = val.x;
    p3.y = val.y;
    p3.z = val.z;
    pts_3d.push_back(p3);
  }

  cout << "cv_camera_matrix: " << cv_camera_matrix << std::endl
       << std::endl;
  cout << "cv_distortion_coeff: " << cv_distortion_coeff << std::endl
       << std::endl;

  cv::projectPoints(pts_3d, cv_r_vec, cv_t_vec, cv_camera_matrix, cv_distortion_coeff, pts_2d);

  std::vector<PnPData> pnp_list_inliner;
  for (size_t i = 0; i < pts_3d.size(); i++)
  {
    const auto du = pts_2d[i].x - pnp_list[i].u;
    const auto dv = pts_2d[i].y - pnp_list[i].v;
    std::cout << i << ":du " << du  << " . dv " << dv << std::endl;
 
    if ( std::fabs(du) < 10  && std::fabs(dv) < 10 )
    {
      pnp_list_inliner.push_back( pnp_list[i] );
    }
  }
  cout << "265 pnp_list_inliner size: " << pnp_list_inliner.size() << std::endl << std::endl;
  return pnp_list_inliner;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidarCamCalib");
  ros::NodeHandle nh;

  // nh.param<string>("common/image_file", image_file, "");
  // nh.param<string>("common/pcd_file", pcd_file, "");
  nh.param<string>("common/result_file", result_file, "");
  std::cout << "pcd_file path:" << pcd_file << std::endl;
  std::cout << "image_file path:" << image_file << std::endl;

  nh.param<vector<double>>("camera/camera_matrix", camera_matrix,
                           vector<double>());
  nh.param<vector<double>>("camera/dist_coeffs", dist_coeffs, vector<double>());
  nh.param<bool>("calib/use_rough_calib", use_rough_calib, false);
  nh.param<string>("calib/calib_config_file", calib_config_file, "");

  cv::Mat cv_camera_matrix =
      (cv::Mat_<double>(3, 3) << camera_matrix[0], 0.0, camera_matrix[2],
       0.0, camera_matrix[4], camera_matrix[5],
       0.0, 0.0, 1.0);
  cv::Mat cv_distortion_coeff =
      (cv::Mat_<double>(1, 5) << dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3], 0.0);

  // R << 0.00212337 ,  -0.999983 , -0.00541819 , 
  //        0.0160035 , 0.00545149 ,  -0.999857 ,
  //        0.99987 , 0.00203635 ,  0.0160148 ;
         
  R << 0.0 , -1.0 ,   0.0 ,
       0.0 ,  0.0 ,  -1.0 ,
       1.0 ,  0.0 ,   0.0 ;
  T << 0.0, 0.0, -0.08 ;

  std::vector<PnPData> pnp_list;
  ROS_INFO_STREAM("Finish prepare!");

  inner << camera_matrix[0], 0.0, camera_matrix[2],
      0.0, camera_matrix[4], camera_matrix[5],
      0.0, 0.0, 1.0;
  distor << dist_coeffs[0], dist_coeffs[1], dist_coeffs[2], dist_coeffs[3];

  // Maximum match distance threshold: 15 pixels
  // If initial extrinsic lead to error over 15 pixels, the algorithm will not
  // work

  std::fstream file;
  std::string filename = "/home/dlvc_data/vel2camera_vanjee_hk/pre/dvl_manual_pnp.txt";
  // std::string filename = "/home/dlvc_data/vel2camera_vanjee_hk/xf_yl/pnp_xf_yl.txt";
  file.open(filename);

  int cnts = 0;
  // while(!file.eof())
  while (cnts < 12 )
  {
    // if(file == "end")  break;
    PnPData tmp;
    file >> tmp.x >> tmp.y >> tmp.z >> tmp.u >> tmp.v;
    tmp.u = int(tmp.u);
    tmp.v = int(tmp.v);
    // tmp.z = 1;
    pnp_list.push_back(tmp);
    cnts++;
    std::cout << tmp.x << " " << tmp.y << " " << tmp.z << " " << tmp.u << " " << tmp.v << std::endl;
  }
  file.close();

  for (size_t i = 0; i < 10; i++)
  {
    if (pnp_list.size() >= 5)
    {
      Eigen::Quaterniond last_q(R);
      Eigen::Vector3d last_t = T;

      solver_2d_3d(pnp_list);

      Eigen::Quaterniond opt_q(R);

      pnp_list = compute_residuals_error(pnp_list, cv_camera_matrix, cv_distortion_coeff);
      
      std::cout <<std::setprecision(6) << "q_dis:" << RAD2DEG(opt_q.angularDistance(last_q)) << " ,t_dis:" << (T - last_t).norm() << std::endl;
      // 旋转增量小于0.01度 and 平移增量小于0.005m 结束优化
      if (opt_q.angularDistance(last_q) < DEG2RAD(0.0001) && (T - last_t).norm() < 0.005) {
        ROS_WARN("DONE --------------------------------------- at %ld th optimization. ", i);
        break;
      }

    }
  }

  return 0;


}