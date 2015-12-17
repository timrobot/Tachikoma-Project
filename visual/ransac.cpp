#include <armadillo>
#include <vector>
#include <cmath>

using namespace arma;
using namespace std;

mat ransac(vector<vec> A) {
  int k = A.size() * 2; // heuristical, less than A.size() * A.size()
  int consensus = 0;
  vec pt1, pt2;
  for (int i = 0; i < k; i++) {
    // choose two random points
    int i1 = rand() % size(A);
    int i2 = rand() % size(A);

    // fit a particular model
    vec u_i2_i1 = normalise(A[i2] - A[i1]);

    // get the set of inliers that are close enough to the line
    int hyp_consensus = 0;
    double inlier_tolerance = 0.1;
    for (int i = 0; i < k; i++) {
      // compute the distance from the point
      vec v_i_i1 = A[i] - A[i1];
      vec para = (v_i_i1.t() * u_i2_i1) * u_i2_i1;
      vec perp = v_i_i1 - para;
      double distance = sqrt(perp.t() * perp);
      if (distance < inlier_tolerance) {
        hyp_consensus++;
      }
    }

    // compare the consensus to the known consensus
    // if larger, then replace
    if (hyp_consensus > consensus) {
      pt1 = A[i1];
      pt2 = A[i2];
      consensus = hyp_consensus;
    }
  }

  mat ans(2, 2);
  ans.col(0) = pt1;
  ans.col(1) = pt2;
  return ans;
}
