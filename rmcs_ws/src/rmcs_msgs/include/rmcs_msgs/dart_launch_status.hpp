namespace rmcs_msgs {
enum class DartLaunchStatus {
    DISABLE = 0,
    INIT = 1,
    LOADING = 2,
    LAUNCH_READY = 3,
};
/*
说明：
    DISABLE：电机全部失能，遥控器双下
    INIT：初始状态，需要滑台复位
    LOADING：上膛和填装中
    LAUNCH_READY：可发射状态。发射完成后进入INIT
特殊：
    LOADING -> INIT：退膛，转动带下去接滑台使其复位，避免空放
*/
} // namespace rmcs_msgs