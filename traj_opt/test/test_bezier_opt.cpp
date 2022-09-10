#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <iostream>

#include "bernstein/bezier_optimizer.hpp"
#include "traj_utils/bernstein.hpp"

namespace traj_opt {
class BezierOptTest : public ::testing::Test {
 protected:
  BezierOptTest() {
    _optimizer.reset(new BezierOpt());
    std::cout << "BezierOptTest" << std::endl;
  }
  ~BezierOptTest() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    /* set up constraints */
    Eigen::MatrixX4d cube;
    cube.resize(6, 4);
    cube <<
        // clang-format off
    -1, 0, 0, 0,
    1,  0, 0, -3,
    0, -1, 0, 0,
    0,  1, 0, -3,
    0,  0, -1, 0,
    0,  0, 1, -3;
    // clang-format on
    std::vector<Eigen::MatrixX4d> safety_corridors;
    safety_corridors.push_back(cube);
    Eigen::Matrix3d start, end;
    start << 1, 1, 1, 0, 0, 0, 0, 0, 0;
    end << 2, 2, 2, 0, 0, 0, 0, 0, 0;
    std::vector<double> t;
    t.push_back(2);

    _optimizer->setup(start, end, t, safety_corridors);
    _optimizer->calcCtrlPtsCvtMat();
  }

  void           TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BezierOpt::Ptr _optimizer;
};

class BezierOptTest2 : public ::testing::Test {
 protected:
  BezierOptTest2() {
    _optimizer.reset(new BezierOpt());
    std::cout << "BezierOptTest2" << std::endl;
  }
  ~BezierOptTest2() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    /* set up constraints */
    Eigen::MatrixX4d cube1, cube2;
    cube1.resize(6, 4);
    cube1 <<
        // clang-format off
    -1, 0, 0, 0,
    1,  0, 0, -3,
    0, -1, 0, 0,
    0,  1, 0, -3,
    0,  0, -1, 0,
    0,  0, 1, -3;
    // clang-format on
    cube2.resize(6, 4);
    cube2 <<
        // clang-format off
    -1, 0, 0, -2,
    1,  0, 0, -5,
    0, -1, 0, -2,
    0,  1, 0, -5,
    0,  0, -1, -2,
    0,  0, 1, -5;
    // clang-format on

    std::vector<Eigen::MatrixX4d> safety_corridors;
    safety_corridors.push_back(cube1);
    safety_corridors.push_back(cube2);
    Eigen::Matrix3d start, end;
    start << 1, 1, 1, 0, 0, 0, 0, 0, 0;
    end << 4, 4, 4, 0, 0, 0, 0, 0, 0;
    std::vector<double> t;
    t.push_back(2);
    t.push_back(2);

    _optimizer->setup(start, end, t, safety_corridors);
    _optimizer->calcCtrlPtsCvtMat();
  }

  void           TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BezierOpt::Ptr _optimizer;
};

TEST_F(BezierOptTest, TestCvtMat) {
  Eigen::MatrixXd p2v, v2a, a2j;
  p2v = _optimizer->getPos2VelMat();
  v2a = _optimizer->getVel2AccMat();
  a2j = _optimizer->getAcc2JerkMat();
  std::cout << "p2v: " << std::endl << p2v << std::endl;
  std::cout << "v2a: " << std::endl << v2a << std::endl;
  std::cout << "a2j: " << std::endl << a2j << std::endl;
  std::cout << "p2a: " << std::endl << v2a * p2v << std::endl;
  EXPECT_EQ(p2v.rows(), 4 * 3);
  EXPECT_EQ(p2v.cols(), 5 * 3);
  EXPECT_EQ(v2a.rows(), 3 * 3);
  EXPECT_EQ(v2a.cols(), 4 * 3);
  EXPECT_EQ(a2j.rows(), 2 * 3);
  EXPECT_EQ(a2j.cols(), 3 * 3);

  Eigen::MatrixXd p2j = a2j * v2a * p2v;
  std::cout << "p2j: " << std::endl << p2j << std::endl;
}

TEST_F(BezierOptTest, TestMinJerkCost) {
  Eigen::MatrixXd Q = _optimizer->getQ();
  std::cout << "cost: " << std::endl << Q << std::endl;
  EXPECT_EQ(Q.rows(), 5 * 3);
  EXPECT_EQ(Q.cols(), 5 * 3);
}

TEST_F(BezierOptTest2, TestOpt) { 
  Eigen::MatrixXd A = _optimizer->getA();
  Eigen::VectorXd b = _optimizer->getb();
  Eigen::VectorXd lb = _optimizer->getlb();
  Eigen::MatrixXd   Ablb;
  Ablb.resize(A.rows(), A.cols() + 2);
  Ablb << A, lb, b;
  // std::cout << "Ablb: " << std::endl << Ablb << std::endl;
  EXPECT_EQ(b.rows(), A.rows());
  EXPECT_EQ(A.cols(), 5 * 3 * 2);
  bool flag = _optimizer->optimize(); 
  EXPECT_TRUE(flag);
  std::cout << "x_:" << std::endl << _optimizer->getOptCtrlPts() << std::endl;
  Eigen::MatrixXd X = _optimizer->getOptCtrlPtsMat();
  std::cout << "X: " << std::endl << X << std::endl;
  EXPECT_EQ(X.rows(), 3);
  EXPECT_EQ(X.cols(), 10);
}

}  // namespace traj_opt

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
