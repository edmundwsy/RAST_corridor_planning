#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <iostream>

#include "traj_utils/bernstein.hpp"

namespace planner {
class BernsteinTest : public ::testing::Test {
 protected:
  BernsteinTest() {
    std::cout << "BernsteinTest" << std::endl;
  }
  ~BernsteinTest() {}
  virtual void SetUp() override {
    std::cout << "enter into SetUp()" << std::endl;
    Eigen::MatrixXd cpts(5, 3);
    cpts << 0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4;
    bp = BernsteinPiece(cpts, 2, 4);
  }
  void TearDown() override { std::cout << "exit from TearDown" << std::endl; }
  BernsteinPiece bp;
};

TEST_F(BernsteinTest, TestCase1) {
  EXPECT_EQ(bp.getDuration(), 2);

  EXPECT_EQ(bp.getPos(2), Eigen::Vector3d::Zero());
  std::cout << bp.getVel(2).transpose() << std::endl;
  EXPECT_EQ(bp.getVel(2), Eigen::Vector3d(2, 2, 2));
  EXPECT_EQ(bp.getAcc(2), Eigen::Vector3d(0, 0, 0));
  EXPECT_EQ(bp.getPos(4), Eigen::Vector3d(4, 4, 4));
}

TEST_F(BernsteinTest, TestControlPoints) {
  Eigen::MatrixXd A;
  A.resize(5, 5);
  A << 1, -4, 6, -4, 1, 0, 4, -12, 12, -4, 0, 0, 6, -12, 6, 0, 0, 0, 4, -4, 0, 0, 0, 0, 1;
  EXPECT_EQ(bp.getCoeffMat(), A);

  Eigen::MatrixXd v_cpts(4, 3);
  v_cpts << 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4;
  std::cout << bp.getVelCtrlPts() << std::endl;
  EXPECT_EQ(bp.getVelCtrlPts(), v_cpts);

  Eigen::MatrixXd a_cpts(3, 3);
  a_cpts << 0, 0, 0, 0, 0, 0, 0, 0, 0;
  EXPECT_EQ(bp.getAccCtrlPts(), a_cpts);

}

TEST_F(BernsteinTest, TestGetPos) {
  for (double t = 2; t < 4; t += 0.1) {
    Eigen::Vector3d p, v, a;
    p = bp.getPos(t);
    v = bp.getVel(t);
    a = bp.getAcc(t);
    std::cout << "t: " << t << ", p: " << p.transpose() << ", v: " << v.transpose()
              << ", a: " << a.transpose() << std::endl;
  }
}
}  // namespace planner

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
