typedef arma::mat (*MotionFcn)(const arma::mat &start, const arma::mat &stop, double t, void *args);

class MotionState {
  // state initial construction
  MotionState(void);
  MotionState(const arma::mat &delta_distance,
      const double &total_time,
      MotionFcn motion_fcn,
      double tolerance = 1.0);

  // Use the frame of reference model
  arma::mat offset_distance; // set by the library
  // intput (can be changed by the user):
  arma::mat delta_distance;
  double total_time;
  MotionFcn motion_fcn;
  double tolerance;
  arma::mat unit_target_motion(const arma::mat &curr_pos); // computational query
  bool finished(const arma::mat &curr_pos);
  // output
  arma::mat unit_motion;
  void *args;
};

class InterMotion {
  Motion(void);
  std::string name;

  // either this
  Motion *motion_root;
  // or this
  InterMotion *parent;

  // either this
  MotionState *state;
  // or this
  std::vector<InterMotion *> submotion;
  std::map<std::string, int> subhash;
  int subindex; // reset and elevate when finished
};

class Motion {
  public:
    MotionState *goal_state;
    MotionState *curr_state; // jump reference
    InterMotion *inter_motion_root; // query reference
    void set_goal(std::string name); // set the query goal as well as (may reset?) the motion feature
};
