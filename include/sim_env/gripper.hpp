

namespace franka{

struct GripperState {
    double width;
    bool   is_grasped;
};

class Gripper{
public:
    Gripper();
    ~Gripper();

    GripperState readOnce();
};


};