#include "race_scar/race_scar.h"

double obstacle_clearance = 4.1;
int planback_count = 0;

// wyc 1019: 预估分数
double calculateScore(const BillStatus* order, Car car) {
    double ordertime, bettertime, timeout;

    ordertime =  order->orderTime / 1000;
    bettertime = order->betterTime / 1000;
    timeout = order->timeout / 1000;

    ros::Time now = ros::Time::now();
    // cout<<now<<endl;
    // cout<<ordertime - now.toSec()<<endl;
    // std::cout << "Press any key to continue..." << std::endl;
    // std::cin.get();
    
    
    double transport_time = 0;
    double takeoff_time = 0;
    // wyc 0929: 细化车俩的余量 
    double transport_go_time = 0;
    double transport_back_time = 0;
    // wyc 1020: 动态余量
    // wyc 1021: 起飞高度变化，动态起飞时间
    switch (car.car_index_)
    {
    case 1:
        transport_time = 50.0;
        transport_back_time = 16.0;
        transport_go_time = 14.0;
        takeoff_time = 20.0;

        break;
    case 2:
        transport_time = 50.0;
        transport_back_time = 16.0;
        transport_go_time = 14.0;
        takeoff_time = 20.0;
        break;
    case 3:
        transport_time = 75.0;
        transport_back_time = 35.0;
        transport_go_time = 26.0;
        takeoff_time = 20.0;
        break;
    case 4:
        transport_time = 75.0;
        transport_back_time = 35.0;
        transport_go_time = 26.0;
        takeoff_time = 20.0;
        break;
    default:
        transport_time = 90.0;
        transport_go_time = 45.0;
        transport_back_time = 45.0;
        takeoff_time = 20.0;
        break;
    }
    double dis_order = (positionToVector2d(car.unloading_point) - positionToVector2d(car.park_point)).norm();
    // 现在时间+车返程+飞机起飞+飞机送货+飞机下降=预估时间
    // wyc todo: 预估等待时间
    double estimatedTime;
    if(now.toSec()>=ordertime){
        estimatedTime = now.toSec() + transport_back_time + 25.0 + (dis_order / 7.5) + 25.0;
    }else{
        // 等待时间*4 惩罚占用
        estimatedTime = now.toSec() + transport_back_time + 25.0 + (dis_order / 7.5) + 25.0 + 4 * ( ordertime - now.toSec());
    }
    
    // 分数：如果estimatedTime小于bettertime，分数为100;如果estimatedTime在bettertime与timeout之间，分数为100/(estimatedTime-bettertime+0.1)*(timeout-bettertime+0.1);如果estimatedTime大于timeout，分数为（-100）/（timeout-bettertime）*（estimatetime-timeout）

    if (estimatedTime < bettertime) {
        // 如果estimatedTime小于betterTime，分数为100
        return 100.0;
    } else if (estimatedTime >= bettertime && estimatedTime <= timeout) {
        // 如果estimatedTime在betterTime与timeout之间
        return 100.0 / (timeout - bettertime) * (timeout - estimatedTime);
    } else {
        // 如果estimatedTime大于timeout
        return -100.0 / (timeout - bettertime) * (estimatedTime - timeout);
    }
}


// wyc 1021: 在loading时预估分数，考虑车动时间
double calculateScore_Load(const BillStatus* order, Car car) {
    double ordertime, bettertime, timeout;

    ordertime =  order->orderTime / 1000;
    bettertime = order->betterTime / 1000;
    timeout = order->timeout / 1000;

    ros::Time now = ros::Time::now();
    // cout<<now<<endl;
    // cout<<ordertime - now.toSec()<<endl;
    // std::cout << "Press any key to continue..." << std::endl;
    // std::cin.get();
    
    
    double transport_time = 0;
    double takeoff_time = 0;
    // wyc 0929: 细化车俩的余量 
    double transport_go_time = 0;
    double transport_back_time = 0;
    // wyc 1020: 动态余量
    // wyc 1021: 起飞高度变化，动态起飞时间
    switch (car.car_index_)
    {
    case 1:
        transport_time = 50.0;
        transport_back_time = 16.0;
        transport_go_time = 14.0;
        takeoff_time = 20.0;

        break;
    case 2:
        transport_time = 50.0;
        transport_back_time = 16.0;
        transport_go_time = 14.0;
        takeoff_time = 20.0;
        break;
    case 3:
        transport_time = 75.0;
        transport_back_time = 35.0;
        transport_go_time = 26.0;
        takeoff_time = 20.0;
        break;
    case 4:
        transport_time = 75.0;
        transport_back_time = 35.0;
        transport_go_time = 26.0;
        takeoff_time = 20.0;
        break;
    default:
        transport_time = 90.0;
        transport_go_time = 45.0;
        transport_back_time = 45.0;
        takeoff_time = 20.0;
        break;
    }
    double dis_order = (positionToVector2d(car.unloading_point) - positionToVector2d(car.park_point)).norm();
    // 现在时间+车返程+飞机起飞+飞机送货+飞机下降=预估时间
    // wyc 1020 todo: 预估等待时间
    double estimatedTime;
    if(now.toSec() + transport_go_time >=ordertime){
        estimatedTime = now.toSec() + transport_back_time + 25.0 + (dis_order / 7.5) + 25.0;
    }else{
        // 等待时间*4 惩罚占用
        estimatedTime = now.toSec() + transport_back_time + 25.0 + (dis_order / 7.5) + 25.0 + 4 * ( ordertime - now.toSec() - transport_go_time);
    }
    
    // 分数：如果estimatedTime小于bettertime，分数为100;如果estimatedTime在bettertime与timeout之间，分数为100/(estimatedTime-bettertime+0.1)*(timeout-bettertime+0.1);如果estimatedTime大于timeout，分数为（-100）/（timeout-bettertime）*（estimatetime-timeout）

    if (estimatedTime < bettertime) {
        // 如果estimatedTime小于betterTime，分数为100
        return 100.0;
    } else if (estimatedTime >= bettertime && estimatedTime <= timeout) {
        // 如果estimatedTime在betterTime与timeout之间
        return 100.0 / (timeout - bettertime) * (timeout - estimatedTime);
    } else {
        // 如果estimatedTime大于timeout
        return -100.0 / (timeout - bettertime) * (estimatedTime - timeout);
    }
}

/**
 * @brief MAIN
*/
void mtuavFSM(const ros::TimerEvent& event) {
    if (current_panoramic_info_) {
        cout << "================================================================================" << endl;
        cout << "================================================================================" << endl;
        cout << "    =========           ==========           ===             ==========         " << endl;
        cout << "   ==                  =                   ==   ==           ==      ==         " << endl;
        cout << "    =========         =                   == === ==          ========           " << endl;
        cout << "            ==         =                 ==       ==         ==     ==          " << endl;
        cout << "    ========            ==========      ==         ==        ==       ==        " << endl;
        cout << "================================================================================" << endl;
        cout << "================================================================================" << endl;
        // wyc 1025: 在展示分数时间的基础上，统计空跑和解邦次数
        // wyc 1025: 时间大于3520秒，程序暂停
        cout << "---total score: " << current_panoramic_info_->score << " time_sec: "<< (ros::Time::now() - time_start_).toSec() 
        << "----count for wrong time: " << wrong_time_cnt_ << " ----count for unbind: " << unbind_cnt_
        <<endl;
        if ((ros::Time::now() - time_start_).toSec() > 3520){
            std::cout << "TASK OVER Press any key to continue..." << std::endl;
            std::cin.get();  
        }
        
    } else {
        cout << "No panoramic info received yet." << endl;
        return;
    }
   
    double transport_time = 0;
    double takeoff_time = 0;
    // wyc 0929: 细化车俩的余量 
    double transport_go_time = 0;
    double transport_back_time = 0;

    for (auto &car : car_vector_) {
        // wyc: 判断车辆位置，决定余量
        switch (car.car_index_)
        {
        case 1:
            transport_time = 50.0;
            transport_back_time = 16.0;
            transport_go_time = 14.0;
            takeoff_time = 20.0;
            break;
        case 2:
            transport_time = 50.0;
            transport_back_time = 16.0;
            transport_go_time = 14.0;
            takeoff_time = 20.0;
            break;
        case 3:
            transport_time = 75.0;
            transport_back_time = 35.0;
            transport_go_time = 26.0;
            takeoff_time = 20.0;
            break;
        case 4:
            transport_time = 75.0;
            transport_back_time = 35.0;
            transport_go_time = 26.0;
            takeoff_time = 20.0;
            break;
        default:
            transport_time = 90.0;
            transport_go_time = 45.0;
            transport_back_time = 45.0;
            takeoff_time = 20.0;
            break;
        }
        car.transport_time_ = transport_time;
        car.transport_go_time_ = transport_go_time;
        car.transport_back_time_ = transport_back_time;
        car.takeoff_time_ = takeoff_time;

        // 动态余量
        double active_dis = transport_time * 10;
        // wyc0929: 细化返程动态余量
        double active_back_dis = transport_back_time * 10;
        double active_go_dis = transport_go_time * 10;

        // wyc 1020:根据顶点ordertime添加等待余量
        double wait_active_dis = max(0.0, (car.ordered_waybill_list_[0]->orderTime / 1000 - ros::Time::now().toSec()) * 10);
        active_back_dis += wait_active_dis;



        cout << "===================================================================================================" << endl;
        cout << "======================================== " << car.sn << " Table ================================" << endl;
        cout << "Car work state: " << car_work_state_str_[int(car.state-1)] << " system statu: " 
             << to_string(car.physical_statu.car_work_state) << " serve index: " <<  car.serve_index << endl;
        for (size_t i = 0; i < car.drones.size(); ++i) {
            cout << "|---" << car.drones[i]->sn << ":" << " work state: " << drone_work_state_str_[int(car.drones[i]->state-1)]
                 << " system statu: " << to_string(car.drones[i]->physical_statu.drone_work_state) 
                 << " battery: " << car.drones[i]->physical_statu.remaining_capacity << endl;
            cout << "--------------------------------------------------------------" << endl;
        }
        cout << "======================================== " << car.sn << " Table ================================" << endl;
        cout << "===================================================================================================" << endl;
        cout << endl;

        // 计算每个车辆负责机组未来路径最短（可以优化，未考虑起降）
        // wyc 0930: 根据余量调整优先级
        // 正在运行版本double minDistance = 300.0;

        double minDistance = 0.0;
        car.closest_back_drone_dis = minDistance;
        for (auto& drone : car.drones) {
            double dis_xy = (positionToVector2d(drone->physical_statu.pos.position) - positionToVector2d(car.park_point)).norm() - active_dis;  // 返程距离
            double dis_go_and_back_xy = 
                (positionToVector2d(car.unloading_point) - positionToVector2d(car.park_point)).norm() 
                + (positionToVector2d(car.unloading_point) - positionToVector2d(drone->physical_statu.pos.position)).norm() + 25 * 10 * 2 - active_dis;  //  去程距离
            double dis_z = abs(drone->physical_statu.pos.position.z - car.park_point.z);  // 高度差
            double dis = dis_xy + dis_z;
            double dis_go_and_back = dis_go_and_back_xy + dis_z;

            if (!car.physical_statu.drone_sn.empty()) {
                // 速度修改
                dis -= (transport_time / 2) * 10;
                dis_go_and_back -= (transport_time / 2) * 10;
                // on car 且 dis比较大的，比普通dis比较大的优先
                minDistance -= 3;
            }

            // 返程
            if((drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0)
                || drone->state == PLAN_BACK     
                || drone->state == UNLODING_CARGO) 
                {
                if (dis < minDistance) {
                    minDistance = dis;
                }
                 car.closest_back_drone_dis = minDistance;
            }
            // 送货
            else if (drone->state == FLYING && drone->physical_statu.bind_cargo_id != 0) {
                if (dis_go_and_back < minDistance) {
                    minDistance = dis_go_and_back;
                }
                car.closest_back_drone_dis = minDistance;
            } else {
                if (minDistance < car.closest_back_drone_dis) {
                    car.closest_back_drone_dis = minDistance;
                }
            }
            // 第一次跑跑最近的4个按顺序
            if(first_time_ == true && (car.car_index_== 1 || car.car_index_== 2 || car.car_index_== 3 || car.car_index_== 4)){
                switch (car.car_index_)
                {
                case 1:
                    car.closest_back_drone_dis -= 1.8;
                    break;
                case 2:
                    car.closest_back_drone_dis -= 1.9;
                    break;
                case 3:
                    car.closest_back_drone_dis -= 0.8;
                    break;
                case 4:
                    car.closest_back_drone_dis -= 0.6;
                    break;
                
                default:
                    car.closest_back_drone_dis -= 0;
                    break;
                }
            }



        }
        // 09/27： 不是第一次才加随机
        if(!first_time_){
            std::random_device rd;  // 非确定性随机数生成器
            std::mt19937 gen(rd()); // 以随机设备初始化Mersenne Twister生成器
            // 创建一个在[0, 1)区间的均匀分布
            std::uniform_real_distribution<> dis(0.0, 1.0);
            // 生成随机小数
            double random_number = dis(gen);
            car.closest_back_drone_dis += random_number;
        }

    }
    // wyc 1020 : 延迟5个心跳，适应车反应慢
    if (load_flag_ == false) {
        loading_busy_ = false;
    }

    // 不是第一次，根据car中的最小距离调整car_vector_顺序，是第一次根据index调整顺序
    if(!first_time_){
        std::sort(car_vector_.begin(), car_vector_.end(), [](const Car& a, const Car& b) {
            return a.closest_back_drone_dis < b.closest_back_drone_dis;});
        cout << "===================================================sorted car======================================" << endl;
        for (auto &car : car_vector_) {
            cout << "Car ID: " << car.sn << ", Closest Drone Distance: " << car.closest_back_drone_dis << endl;
        }
        cout << "===================================================================================================" << endl;
    }
    // 是第一次，按照index从小到大
    else{
        std::sort(car_vector_.begin(), car_vector_.end(), [](const Car& a, const Car& b) {
        return a.car_index_ < b.car_index_;});
        cout << "===================================================sorted car======================================" << endl;
        for (auto &car : car_vector_) {
            cout << "First Car ID: " << car.sn << ", Closest Drone Distance: " << car.closest_back_drone_dis << endl;
        }
        cout << "===================================================================================================" << endl;
        first_time_ = false;
        // car_index前四架飞机都飞完了，first_time 置否
        // wyc 1019: 四机版本，不要56飞机
        // if(car_vector_[0].first == false && car_vector_[1].first == false && car_vector_[2].first == false && car_vector_[3].first == false){
        //     first_time_ = false;
        //     cout << "================================================================================================" << endl;
        //     cout << "===================================================new era======================================" << endl;
        //     cout << "===================================================pls stop peeping!!===========================" << endl;
        //     cout << "===================================================pls stop peeping!!===========================" << endl;
        //     cout << "================================================================================================" << endl;
        //     cout << "================================================================================================" << endl;
        // }

    }

    
    // ==================== Main loop ====================    
    for (auto &car : car_vector_) {

        // wyc: 判断车辆位置，决定余量
        // takeoff time 是上升到返程高度的时间
        switch (car.car_index_)
        {
        case 1:
            transport_time = 50.0;
            transport_back_time = 16.0;
            transport_go_time = 16.0;
            takeoff_time = 20.0;

            break;
        case 2:
            transport_time = 50.0;
            transport_back_time = 16.0;
            transport_go_time = 16.0;
            takeoff_time = 20.0;
            break;
        case 3:
            transport_time = 70.0;
            transport_back_time = 41.0;
            transport_go_time = 28.0;
            takeoff_time = 20.0;
            break;
        case 4:
            transport_time = 70.0;
            transport_back_time = 41.0;
            transport_go_time = 28.0;
            takeoff_time = 20.0;
            break;
        default:
            transport_time = 90.0;
            transport_go_time = 45.0;
            transport_back_time = 45.0;
            takeoff_time = 20.0;
            break;
        }
        // wyc 1020: 考虑订单出现时间
        double wait_active_dis = max(0.0, (car.ordered_waybill_list_[0]->orderTime / 1000 - ros::Time::now().toSec()) * 10);
        // 动态余量
        double active_dis = transport_time * 10;
        // wyc0929: 细化返程动态余量
        double active_back_dis = transport_back_time * 10;
        double active_go_dis = transport_go_time * 10;
        double take_off_dis = takeoff_time * 10;
        active_back_dis += wait_active_dis;

        // 飞机状态判断，用于决定车辆服务哪一架飞机
        std::vector<int> emergencyScore(car.drones.size());
        // 执行顺序：正在执行装货充电 > 返程很近 > 接init执行 > 返程很远 > 其他
        for (size_t i = 0; i < car.drones.size(); ++i) {
            switch (car.drones[i]->state) {
                case INIT: {
                    // cout << "--Waitting car~" << endl;
                    // 如果这个飞机是server的，进入，否则就啥也不干
                    if(i == car.serve_index){
                        if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_READY) {
                            double dis = (positionToVector3d(car.physical_statu.pos.position) - positionToVector3d(loading_cargo_point_)).norm();
                            if (dis < 0.5) {
                                car.drones[i]->state = BIND_CAR;
                            }else{
                                car.state = PLAN_LOADING;
                            }
                        }
                    }
                    break;
                }

                case BIND_CAR: {
                    if (!car.physical_statu.drone_sn.empty()) {
                        car.drones[i]->state = LOADING_CARGO;
                        break;
                    }
                    
                    // 如果绑定车的时候发现情况不对，不继续绑定车
                    bool right_time = true;

                    // wyc 1008: 如果oncar的飞机电量少于30，则charge and cargo时间增加换电时间
                    double charge_and_cargo = 10.0 * 10.0;
                    if (car.drones[i]->physical_statu.remaining_capacity < 30.0) {
                        charge_and_cargo = 20.0 * 10.0;
                    }


                    for (auto & drone : car.drones){
                        double dis = (positionToVector2d(drone->physical_statu.pos.position) - 
                                      positionToVector2d(car.park_point)).norm();

                        // wyc 1021: 判断条件修改，增加去程过近的判断
                        double dis_unload = (positionToVector2d(drone->physical_statu.pos.position) - 
                                      positionToVector2d(car.unloading_point)).norm();
                        //   wyc 1021: 飞机高度到地距离 + 1.5 是算上加减速的估计，可以用准确时间优化；15*10是数了的5米悬停+降落时间
                        double t_height_down = abs(drone->physical_statu.pos.position.z) * 1.5 + 15 * 10;


                        user_pkg::UserCmdRequest msg;
                        msg.peer_id = config_.getPeerId();
                        msg.task_guid = config_.getGuid();
                        // wyc:0929: 细化绑车条件
                        // 绑车如果发现车回去起飞动态余量来不及（返程飞机低于动态余量  或  卸货准备起飞飞机低于动态余量+起飞时间）
                        // wyc 1007: 优化解邦条件，考虑飞机起飞时,还没起飞的部分
                        // wyc 1021: t_height backpath[2]超出范围，改为0
                        // wyc 1021: 判断条件修改，增加去程过近的判断
                        // wyc 1025: 由于返程飞机起飞条件修改为必须是对应车上飞机上了货，所以不需要余量了
                        double t_height =  car.drones[i]->back_path[0].z() - drone->physical_statu.pos.position.z;
                        if( drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0 && dis < (active_back_dis + charge_and_cargo  + take_off_dis - t_height))
                                    // ((drone->state== UNLODING_CARGO || drone->state== PLAN_BACK)  && dis < (active_back_dis + charge_and_cargo + take_off_dis - take_off_dis)) ||
                                    // (drone->state == FLYING && drone->physical_statu.bind_cargo_id != 0 && dis < (active_back_dis + charge_and_cargo + take_off_dis + 5 * 10 - dis_unload -25 * 10 -t_height_down))
                                    // )
                        {
                            // wyc 1021: 不邦车就直接安排回去
                            car.drones[i]->state = INIT;
                            car.state = PLAN_PARKING;
                            right_time = false;
                            break;
                        } 
                    }
                    if(!right_time){
                        cout << "BIND_CAR --not right time, not binding car" << endl;
                        car.drones[i]->state = INIT;
                        break;
                    }      

                    UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = UserCmdRequest::USER_CMD_MOVE_DRONE_ON_CAR;
                    msg.binding_drone.car_sn = car.sn;
                    msg.binding_drone.drone_sn = car.drones[i]->sn;
                    cmd_pub_.publish(msg);

                    cout << "--Publishing bind car and drone msg: " << "car sn: " << msg.binding_drone.car_sn 
                         << "drone sn: " << msg.binding_drone.drone_sn << endl;

                    break;
                }

                case LOADING_CARGO: {
                    if (car.physical_statu.drone_sn.empty()) {
                        car.drones[i]->state = INIT;
                        car.state = PLAN_PARKING;
                        break;
                    }

                    // 判断是否需要解邦车
                    bool right_time = true;
                    if(car.drones[i]->physical_statu.bind_cargo_id == 0){
                        for (auto &drone : car.drones){
                            // wyc 1007: 修改dis计算为其他飞机而不是当前机
                            double dis = (positionToVector2d(drone->physical_statu.pos.position) -
                                    positionToVector2d(car.park_point)).norm();
                            user_pkg::UserCmdRequest msg;
                            msg.peer_id = config_.getPeerId();
                            msg.task_guid = config_.getGuid();
                            // wyc 1007: 优化解邦条件，考虑飞机起飞时,还没起飞的部分
                            // wyc 1021: 判断条件修改，增加去程过近的判断
                            double dis_unload = (positionToVector2d(drone->physical_statu.pos.position) - 
                                      positionToVector2d(car.unloading_point)).norm();
                            //   wyc 1021: 飞机高度到地距离 + 1.5 是算上加减速的估计，可以用准确时间优化；15*10是数了的5米悬停+降落时间
                            double t_height_down = abs(drone->physical_statu.pos.position.z) * 1.5 + 15 * 10;

                            // wyc 1021: t_height backpath[2]超出范围，改为0
                            double t_height =  abs(car.drones[i]->back_path[0].z() - drone->physical_statu.pos.position.z) * 1.5;
                            // wyc 1008: 如果oncar的飞机电量少于30，则charge and cargo时间增加换电时间
                            double charge_and_cargo = 10.0 * 10.0;
                            if (car.drones[i]->physical_statu.remaining_capacity < 30.0) {
                                charge_and_cargo = 20.0 * 10.0;
                            }

                            // wyc 1021: 检查解邦条件
                            if(drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0){
                                cout<< "car.drones[i]->back_path[2].z() " << car.drones[i]->back_path[2].z() << endl;
                                cout<< "drone->physical_statu.pos.position.z " << drone->physical_statu.pos.position.z << endl;
                                cout<< "active_back_dis " << active_back_dis << endl;
                                cout<< "charge_and_cargo " << charge_and_cargo << endl;
                                cout<< "take_off_dis " << take_off_dis << endl;
                                cout<< "t_height " << t_height << endl;
                                cout<<" flyback threshold = " << (active_back_dis + charge_and_cargo  + take_off_dis - t_height) << endl;
                            }else if ((drone->state== UNLODING_CARGO || drone->state== PLAN_BACK)){
                                cout<< "active_back_dis " << active_back_dis << endl;
                                cout<< "charge_and_cargo " << charge_and_cargo << endl;
                                cout<< "take_off_dis " << take_off_dis << endl;
                                cout<< "t_height " << t_height << endl;
                                cout << "plan back threshold = " << (active_back_dis + charge_and_cargo + take_off_dis - take_off_dis) << endl;
                            }


                            // wyc 1021: 判断条件修改，增加去程过近的判断
                            // wyc 1025: 由于飞机十分充足，如果电量不足，直接解邦飞机并踢出car对应的飞机列表，从满电的飞机中选择一辆替换进来
                            // wyc 1025: 由于返程飞机增加plan back判断，此处不用再考虑余量
                            // if(drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0 && dis < (active_back_dis + charge_and_cargo  + take_off_dis - t_height))
                            //         //    ((drone->state== UNLODING_CARGO || drone->state== PLAN_BACK)  && dis < (active_back_dis + charge_and_cargo + take_off_dis - take_off_dis)) ||
                            //             // (drone->state == FLYING && drone->physical_statu.bind_cargo_id != 0 && dis < (active_back_dis + charge_and_cargo + take_off_dis + 5 * 10 - dis_unload -25 * 10 -t_height_down))
                            //             // )
                            //     {
                            //     right_time = false;
                            //     wrong_time_cnt_++;
                            //     msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_DRONE_ON_BIRTHPLACE;
                            //     msg.unbind_info.drone_sn = car.drones[i]->sn;
                            //     msg.unbind_info.car_sn = car.sn;
                            //     cmd_pub_.publish(msg);

                            //     // wyc 1025：如果要解邦car.drone[i]的电量少于50，将其剔除car的列表，在未用飞机中随机选一个满电的init飞机替换位置
                            //     if(car.drones[i]->physical_statu.remaining_capacity < 52){
                            //         bool replace_drone = false;
                                    
                            //         // 在所有飞机中找到一个init并且满电的飞机,且这个飞机不能是别的车的drone列表里的
                            //         if (!replace_drone){
                            //             for (auto &drone_backup : drone_vector_){
                            //                 bool drone_is_used = false;
                            //                 for(auto &car_replace : car_vector_){
                            //                     for(auto &drone_in_car : car_replace.drones){
                            //                         if(drone_backup.sn == drone_in_car->sn){
                            //                             drone_is_used = true;
                            //                             break;
                            //                         }  
                            //                     }
                            //                 }
                            //                 // cout<< "drone_is_used " << drone_is_used<< endl;
                            //                 // cout<< "drone_backup.state " << drone_backup.state<< endl;
                            //                 // cout<< "drone_backup.physical_statu.remaining_capacity " << drone_backup.physical_statu.remaining_capacity<< endl;
                            //                 if(!drone_is_used && drone_backup.state == INIT && drone_backup.physical_statu.remaining_capacity > 99){
                            //                     cout << "car.drones[i]->physical_statu.remaining_capacity < 50, replace it with drone_ " << drone_backup.sn << endl;
                            //                     // wyc 1025: 要把计算好的路径也替换进来
                            //                     drone_backup.go_path = car.drones[i]->go_path;
                            //                     drone_backup.back_path = car.drones[i]->back_path;
                            //                     car.drones[i] = &drone_backup;
                            //                     replace_drone = true;
                            //                     break;
                            //                 }  
                            //             }                                        
                            //         }
                            //     }



                            //     cout << "--Publishing unbind cargo msg:" << " drone sn: " << msg.unbind_info.drone_sn
                            //         << " dis = " << dis
                            //         << " threshold = " << (active_back_dis + take_off_dis - t_height)
                            //         << " car sn: " << msg.unbind_info.car_sn 
                            //         << " count for unbind: " << unbind_cnt_++
                            //         << endl;
                            //     // std::cout << "Press any key to continue..." << std::endl;
                            //     // std::cin.get();  
                            //     break;
                            // } 
                        }
                    }
      

                    // wyc 1021: 对waybilllist前10个单进行分数判断,如果订单出现且负数的话从waybilllist中删除
                    if(!car.waybill_list.empty()){
                        for (size_t i = 0; i < min(10, int(car.waybill_list.size())); ++i) {
                            double score = calculateScore(car.waybill_list[i], car);
                            if (score < 0 && (ros::Time::now().toSec() - car.waybill_list[i]->orderTime / 1000) > 0) {
                                car.waybill_list.erase(car.waybill_list.begin() + i);
                            }
                        }
                    }
                    






                    // wyc 1019: 判断是否出现，根据（估算时间-bettertime）来粗略排序，决定上哪个货
                    // 
                    // cout << car.waybill_list[0]->orderTime << endl;
                    car.ordered_waybill_list_ = car.waybill_list;     
                    std::sort(car.ordered_waybill_list_.begin(), car.ordered_waybill_list_.end(),
                        [](auto a, auto b) {
                            return a->orderTime < b->orderTime;
                        });

                    // wyc 1020: 如果最优的都等待时间过长，让它离开,镜像中的时间是ms级
                    cout<< ros::Time::now().toSec() - (car.ordered_waybill_list_[0]->orderTime / 1000) <<endl;
                    // std::cout << "Press any key to continue..." << std::endl;
                    // std::cin.get();
                    if(ros::Time::now().toSec() - (car.ordered_waybill_list_[0]->orderTime / 1000) < -5){
                        right_time = false;
                    }
                    
                    if(!right_time){
                        // cout << "LOADING_CARGO --not right time, not binding car" << endl;
                        // wyc 1025: load 处的非right time 统计
                        cout << "count for wrong load time: " << wrong_time_cnt_ << endl; 
                        break;
                    }    

                    // 程序暂停，等待用户输入
                    // for (size_t i = 0; i < car.ordered_waybill_list_.size(); ++i) {
                    //     std::cout << "Element " << i << ": Order Time: " << car.ordered_waybill_list_[i]->orderTime << ": timeout Time: " << car.ordered_waybill_list_[i]->timeout << "ros_time_now"<< ros::Time::now() << ", Index: " << car.ordered_waybill_list_[i]->index << std::endl;
                    // }

                    // std::cout << "Press any key to continue..." << std::endl;
                    // std::cin.get();







                    // wyc 1019: 对粗略排序后的前10个订单进行分数预估，选择分数最高的订单index
                    std::sort(car.ordered_waybill_list_.begin(), (car.ordered_waybill_list_.begin() + std::min(10, (int)car.ordered_waybill_list_.size())),
                        [&car](const BillStatus* a, const BillStatus* b) {
                            double scoreA = calculateScore(a, car);
                            double scoreB = calculateScore(b, car);
                            return scoreA > scoreB;
                        }
                    );



                   
              


                    // 程序暂停，等待用户输入
                    // for (size_t i = 0; i < car.ordered_waybill_list_.size(); ++i) {
                    //     std::cout << "Element " << i << ": Order Time: " << car.ordered_waybill_list_[i]->orderTime << "ros_time_now"<< ros::Time::now() << ", Index: " << car.ordered_waybill_list_[i]->index << std::endl;
                    // }

                    // std::cout << "Press any key to continue..." << std::endl;
                    // std::cin.get();



                    // wyc 1019: 确保车、机全部ready之后再继续运动
                    if (car.drones[i]->physical_statu.bind_cargo_id != 0 && car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::READY && car.physical_statu.car_work_state == CarPhysicalStatus::CAR_READY) {
                        if (car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::READY) {
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                            cout << "test ready!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                        }

                        // wyc 1019: 准备送的订单删掉
                        // wyc 1021: 修改删除判断为车上飞机绑定的订单
                        // wyc 1021: 修改,删除订单包含orderlist中的订单
                        int cargo_id_to_delete = car.drones[car.serve_index]->physical_statu.bind_cargo_id;

                        // 使用 erase-remove 惯用法来删除特定 index 的订单
                         auto new_end = std::remove_if(car.waybill_list.begin(), car.waybill_list.end(),
                        [cargo_id_to_delete](const BillStatus* bill) {
                            return bill->index == cargo_id_to_delete;
                        });
                        // 实际删除元素
                        car.waybill_list.erase(new_end, car.waybill_list.end());

                        // 使用 erase-remove 惯用法来删除特定 index 的订单
                         auto order_new_end = std::remove_if(car.ordered_waybill_list_.begin(), car.ordered_waybill_list_.end(),
                        [cargo_id_to_delete](const BillStatus* bill) {
                            return bill->index == cargo_id_to_delete;
                        });
                        // 实际删除元素
                        car.ordered_waybill_list_.erase(order_new_end, car.ordered_waybill_list_.end());

                        // car.waybill_count++;


                        //  wyc： 可以优化 通过判断与终点距离
                        
                        if (car.drones[i]->physical_statu.remaining_capacity > 30.0) {
                            car.drones[i]->state = ON_CAR;
                            car.state = PLAN_PARKING;
                        } else {
                            car.drones[i]->state = CHARGE;
                        }
                        break;
                        }
                    // 未绑定货物
                    UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_MOVE_CARGO_IN_DRONE;
                    // wyc 1019: 变为送排序第一个的货物
                    msg.binding_cargo.cargo_id = car.ordered_waybill_list_[0]->index;

                    


                    // msg.binding_cargo.cargo_id = car.waybill_list[car.waybill_count]->index;
                    msg.binding_cargo.drone_sn = car.drones[i]->sn;
                    cmd_pub_.publish(msg);


                    // 程序暂停，等待用户输入
                    // for (size_t i = 0; i < 10; ++i) {
                    //     std::cout << "Element " << i << ": now Time - Order Time: " << (ros::Time::now().toSec() - (car.ordered_waybill_list_[i]->orderTime / 1000)) << ", Index: " << car.ordered_waybill_list_[i]->index << " pred_score: "<< calculateScore(car.ordered_waybill_list_[i],car) << std::endl;
                    // }

                    // std::cout << "Press any key to continue..." << std::endl;
                    // std::cin.get();


                    cout << "--Publishing bind cargo msg:" << " drone sn: " << msg.binding_cargo.drone_sn
                        << " cargo_id: " << msg.binding_cargo.cargo_id << endl;

                    break;
                }

                case ON_CAR: {
                    // cout << "--On car" << endl;
                    break;
                }

                case PLAN_GO: {
                    bool right_time = true;
                    bool too_close = false;

                    for (auto & drone : car.drones){
                        double dis = (positionToVector2d(car.drones[i]->physical_statu.pos.position) -
                                    positionToVector2d(car.park_point)).norm();
                        double dis_order = (positionToVector2d(car.unloading_point) - positionToVector2d(car.park_point)).norm();
                        
                        // 用动态余量判断起飞
                        // wyc 0929:动态余量调整，算上起飞的三维距离，起飞距离可以优化
                        if (dis_order <= 400){
                            too_close = true;
                        }
                        else if(dis_order >= 620){
                            dis_order = 620;
                        }
                        if(drone->state == FLYING && drone->physical_statu.bind_cargo_id != 0 && dis < dis_order + 1 && (car.drones[i]->physical_statu.pos.position.z < -5.0 || too_close)){
                            right_time = false;
                            break;
                        }
                    }
                    if(!right_time){
                        // cout << "--interval too short, wait for plan go" << endl;
                        break;
                    }      
                    if (car.drones[i]->planning) {
                        if (car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::TAKEOFF
                            || car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::FLYING) {
                            car.drones[i]->state = FLYING;
                            car.drones[i]->planning = false;
                        }
                        break;
                    } 

                    UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
                    msg.drone_way_point_info.droneSn = car.drones[i]->sn;
                    // 起飞
                    user_pkg::DroneWayPoint takeoff_point;
                    takeoff_point.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
                    takeoff_point.timeoutsec = 1000;
                    msg.drone_way_point_info.way_point.push_back(takeoff_point);

                    user_pkg::DroneWayPoint flying_point;
                    flying_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
                    flying_point.v = 10.0;
                    flying_point.timeoutsec = 1000;
                    flying_point.pos.x = car.drones[i]->go_path[0].x();
                    flying_point.pos.y = car.drones[i]->go_path[0].y();
                    flying_point.pos.z = car.drones[i]->go_path[0].z();
                    msg.drone_way_point_info.way_point.push_back(flying_point);

                    // 平飞
                    for (int j = 1; j < car.drones[i]->go_path.size(); j++) {
                        if ((car.drones[i]->go_path[j] - car.drones[i]->go_path[j-1]).norm() < 10.0) {
                            flying_point.v = 6.0;
                        } else {
                            flying_point.v = 10.0;
                        }
                        flying_point.pos.x = car.drones[i]->go_path[j].x();
                        flying_point.pos.y = car.drones[i]->go_path[j].y();
                        flying_point.pos.z = car.drones[i]->go_path[j].z();
                        msg.drone_way_point_info.way_point.push_back(flying_point);
                    }

                    // 降落
                    flying_point.v = 10.0;
                    flying_point.pos.z = car.unloading_point.z - 5;
                    msg.drone_way_point_info.way_point.push_back(flying_point);

                    user_pkg::DroneWayPoint landing_point;
                    landing_point.type = user_pkg::DroneWayPoint::POINT_LANDING;
                    landing_point.timeoutsec = 1000;
                    msg.drone_way_point_info.way_point.push_back(landing_point);

                    // wyc 1025: 飞机ready才发布飞行计划
                    if(car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::READY){
                        cmd_pub_.publish(msg);
                        car.drones[i]->planning = true;
                    }


                    cout << "--Publishing drone go flying msg:" << " drone sn: " << msg.drone_way_point_info.droneSn
                         << " drone way point info: " << msg.drone_way_point_info;

                    break;
                }

                case FLYING: {
                    if (car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::READY) {
                        double dis = (positionToVector3d(car.drones[i]->physical_statu.pos.position) - 
                                    positionToVector3d(car.park_point)).norm();
                        if (dis > 0.5) {
                            // cout << "-" << car.sn << " arrive unloading point"<< endl;
                            car.drones[i]->state = UNLODING_CARGO;
                        } else {
                            // cout << "-" << car.sn << " arrive landing point"<< endl;
                            car.state = PLAN_LOADING;
                            car.drones[i]->state = ON_CAR;
                        }
                    }
                    break;
                }

                case UNLODING_CARGO: {
                    if (car.drones[i]->physical_statu.bind_cargo_id == 0) {
                        car.drones[i]->state = PLAN_BACK;
                        break;
                    }

                    user_pkg::UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_RELEASE_CARGO;
                    msg.drone_msg.drone_sn = car.drones[i]->sn;
                    cmd_pub_.publish(msg);

                    // cout << "--test drone cargo index: " << car.drones[i]->physical_statu.bind_cargo_id << endl;
                    cout << "--Publishing drone unloading cargo msg:" << " drone sn: " <<  msg.drone_msg.drone_sn << endl;

                    break;
                }

                case PLAN_BACK: {

                    //wyc 1025: 修改返程条件：
                    // 另一架飞机是INIT 不返程
                    // 另一架是oncar且没有货物，不返程
                    // 另一架在上货或者充电，不返程
                    // 12号近的，上了货的，没规划返程之前不返程
                    // 34号远的，上了货的，没到起飞点之前不返程
                    cout << to_string(car.drones[(i+1)%2]->state) << endl;
                    if( (car.drones[(i+1)%2]->state == INIT) ||
                        (car.drones[(i+1)%2]->state == ON_CAR && car.drones[(i+1)%2]->physical_statu.bind_cargo_id==0) ||
                        (car.drones[(i+1)%2]->state == LOADING_CARGO || car.drones[(i+1)%2]->state == CHARGE) ||
                        ((car.car_index_==1 || car.car_index_==2) && car.drones[(i+1)%2]->physical_statu.bind_cargo_id!=0 && car.state != MOVING) ||
                        ((car.car_index_==3 || car.car_index_==4) && car.drones[(i+1)%2]->physical_statu.bind_cargo_id!=0 && car.state != AT_PARKING_POINT) ||
                        (car.drones[(i+1)%2]->state == FLYING && car.drones[(i+1)%2]->physical_statu.bind_cargo_id==0)
                    )
                    {
                        cout << to_string(car.drones[(i+1)%2]->state) << endl;

                        cout << "NOOOOOOOOOOOOOOOOOOOO PLAN PARK" << endl;
                        break;
                    }
                    // std::cout << "Press any key to continue..." << std::endl;
                    // std::cin.get();

                    // 如果被服务的飞机在的卸货点特别近，叫车回PARKING点
                    double dis = (positionToVector3d(car.drones[i]->physical_statu.pos.position) - 
                                    positionToVector3d(car.park_point)).norm();
                    double dis_car_and_park = (positionToVector3d(car.physical_statu.pos.position) - 
                                                positionToVector3d(car.park_point)).norm();
                    // 如果在parking point 直接改状态，如果不在，让车回parking
                    // wyc 1007: 更新解决bug 返程飞机对应的车刚刚准备动，moving和at parking point状态之间出现冲突
                    // if(i == car.serve_index && dis_car_and_park < 0.5 && car.planning == false && car.state != MOVING)
                    // {
                    //     car.state = AT_PARKING_POINT;
                    // }else if(i == car.serve_index && dis_car_and_park > 0.5 && car.state == AT_PARKING_POINT){
                    //     car.state = PLAN_PARKING;
                    // }


                    if (car.drones[i]->planning) {
                        if (car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::TAKEOFF
                            || car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::FLYING) {
                            car.drones[i]->state = FLYING;
                            car.drones[i]->planning = false;
                        }
                        break;
                    } 
                    
                    UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_EXEC_ROUTE;
                    msg.drone_way_point_info.droneSn = car.drones[i]->sn;

                    // 起飞
                    user_pkg::DroneWayPoint takeoff_point;
                    takeoff_point.type = user_pkg::DroneWayPoint::POINT_TAKEOFF;
                    takeoff_point.timeoutsec = 1000;
                    msg.drone_way_point_info.way_point.push_back(takeoff_point);

                    user_pkg::DroneWayPoint flying_point;
                    flying_point.type = user_pkg::DroneWayPoint::POINT_FLYING;
                    flying_point.v = 10.0;
                    flying_point.timeoutsec = 1000;
                    flying_point.pos.x = car.drones[i]->back_path[0].x();
                    flying_point.pos.y = car.drones[i]->back_path[0].y();
                    flying_point.pos.z = car.drones[i]->back_path[0].z();
                    msg.drone_way_point_info.way_point.push_back(flying_point);

                    // 平飞
                    for (int j = 1; j < car.drones[i]->back_path.size(); j++) {
                       if ((car.drones[i]->back_path[j] - car.drones[i]->back_path[j-1]).norm() < 10.0) {
                            flying_point.v = 6.0;
                        } else {
                            flying_point.v = 10.0;
                        }
                        flying_point.pos.x = car.drones[i]->back_path[j].x();
                        flying_point.pos.y = car.drones[i]->back_path[j].y();
                        flying_point.pos.z = car.drones[i]->back_path[j].z();
                        msg.drone_way_point_info.way_point.push_back(flying_point);
                    }
                    msg.drone_way_point_info.way_point[msg.drone_way_point_info.way_point.size()-1].pos.x = car.park_point.x;
                    msg.drone_way_point_info.way_point[msg.drone_way_point_info.way_point.size()-1].pos.y = car.park_point.y;

                    // 降落
                    flying_point.v = 10.0;
                    flying_point.pos.z = car.park_point.z - 5;
                    msg.drone_way_point_info.way_point.push_back(flying_point);

                    user_pkg::DroneWayPoint landing_point;
                    landing_point.type = user_pkg::DroneWayPoint::POINT_LANDING;
                    landing_point.timeoutsec = 1000;
                    msg.drone_way_point_info.way_point.push_back(landing_point);


                    // wyc 1025: 飞机ready再发布返程轨迹
                    if (car.drones[i]->physical_statu.drone_work_state == DronePhysicalStatus::READY) {
                        cmd_pub_.publish(msg);
                        car.drones[i]->planning = true;
                    }


                    cout << "--Publishing drone go flying msg:" << " drone sn: " << msg.drone_way_point_info.droneSn
                         << " drone way point info: " << msg.drone_way_point_info;

                    break;
                }
            
                case CHARGE: {
                    if (car.drones[i]->physical_statu.remaining_capacity > 95.0) {
                        car.drones[i]->state = ON_CAR;
                        car.state = PLAN_PARKING;
                    }

                    UserCmdRequest msg;
                    msg.peer_id = config_.getPeerId();
                    msg.task_guid = config_.getGuid();
                    msg.type = user_pkg::UserCmdRequest::USER_CMD_DRONE_BATTERY_REPLACEMENT;
                    msg.drone_msg.drone_sn = car.drones[i]->sn;
                    cmd_pub_.publish(msg);

                    cout << "---Publishing drone charge msg:" << endl;
                    cout << "-drone sn: " << msg.drone_msg.drone_sn << endl;

                    break;
                }
            }             
            
            // cout << "--------------------------------------------------------" << endl;
            // cout << "---emergencyScore: " << endl;
            // ON CAR


            
            int case_serve = -1;
            // double dis = (positionToVector3d(car.drones[i]->physical_statu.pos.position) - 
            //               positionToVector3d(car.park_point)).norm();

            double dis_xy = (positionToVector2d(car.drones[i]->physical_statu.pos.position) - positionToVector2d(car.park_point)).norm();  // 返程距离
            double dis_z = abs(car.drones[i]->physical_statu.pos.position.z - car.park_point.z);  // 高度差
            double dis = dis_xy + dis_z;

            double dis_brith = (positionToVector2d(car.drones[i]->physical_statu.pos.position) - 
                          positionToVector2d(birth_pos_)).norm();
            double dis_order = (positionToVector2d(car.unloading_point) - 
                                positionToVector2d(car.park_point)).norm();
            cout <<"car_sn"<< car.sn <<"-------------------------------------------------------"<< endl;
            // cout <<"drone"<< i <<"---work state: " << drone_work_state_str_[int(car.drones[i]->state-1)] << endl;
            // cout <<"drone"<< i << "---dis" << dis << endl;
            // cout <<"drone"<< i << "---dis_brith" << dis_brith << endl;
            // wyc 1021: t_height backpath[2]超出范围，改为0
            double t_height =  abs(car.drones[i]->back_path[0].z() - car.drones[i]->physical_statu.pos.position.z);
            // wyc 1008: 如果oncar的飞机电量少于30，则charge and cargo时间增加换电时间
            double charge_and_cargo = 10.0 * 10.0;

            // wyc 1008: 更新serve判断
            if (car.drones[i]->state != PLAN_BACK && car.drones[i]->state != FLYING && car.drones[i]->state != INIT 
                && car.drones[i]->state != UNLODING_CARGO && car.drones[i]->state != BIND_CAR) {
                case_serve = 0;
                emergencyScore[i] = 2000;
            }
            // flying 返程 or 近点规划返程 
            else if ((car.drones[i]->state == FLYING && car.drones[i]->physical_statu.bind_cargo_id == 0)) {
                // 根据车辆行驶时间，变化余量，余量以内的飞机会先规划降落
                case_serve = 1;

                if ((car.drones[car.serve_index]->physical_statu.remaining_capacity) < 30.0) {
                    charge_and_cargo = 20.0 * 10.0;
                }
                // wyc 1008: 如果车的状态是AT_LOADING_POINT或MOVING，车的位置是在loading point，动态余量不需要考虑active_go_dis
                if(car.state == AT_LOADING_POINT || car.state == MOVING || car.state == PLAN_PARKING){
                    emergencyScore[i] = 400 + charge_and_cargo + active_back_dis + take_off_dis - t_height - dis;
                }else{
                    emergencyScore[i] = 400 + active_go_dis + charge_and_cargo + active_back_dis + take_off_dis - t_height - dis;
                }
                
            }

            //  wyc 1007: 订单太近调整
             // wyc 1025: 返程判断优化所以这里init优先于plan back
            // else if ((car.drones[i]->state== UNLODING_CARGO || car.drones[i]->state== PLAN_BACK)) {
            //     // 根据车辆行驶时间，变化余量，余量以内的飞机会先规划降落
            //     case_serve = 2;
            //     if ((car.drones[i]->physical_statu.remaining_capacity - (dis_xy / 10)) < 40.0) {
            //         charge_and_cargo = 20.0 * 10.0;
            //     }
            //     // wyc 1008: 如果车的状态是AT_LOADING_POINT或MOVING，车的位置是在loading point，动态余量不需要考虑active_go_dis
            //     if(car.state == AT_LOADING_POINT || car.state == MOVING || car.state == PLAN_PARKING){
            //         emergencyScore[i] = 400 + charge_and_cargo + active_back_dis + take_off_dis - take_off_dis - dis;
            //     }else{
            //         emergencyScore[i] = 400 + active_go_dis + charge_and_cargo + active_back_dis + take_off_dis - take_off_dis - dis;
            //     }      
               
            // }


            // 去程但订单太近
            // wyc 0929: 改进，目前不需要考虑订单太近
            // else if((car.drones[i]->state == FLYING && car.drones[i]->physical_statu.bind_cargo_id != 0 && dis_order < (active_dis / 2))){
            //     case_serve = 2;
            //     emergencyScore[i] = 400 + 2 * dis_order - dis - take_off_dis - active_dis;
            // }
            // 订单距离足够远，去程走了很久的优先级应该低于INIT，DIS上升，分数应该变低
            else if(car.drones[i]->state == FLYING && car.drones[i]->physical_statu.bind_cargo_id != 0){
                // 返程距离 可以用车的速度调整
                case_serve = 3;
                double back_dis = (dis_order - dis_xy) + dis_order + 2 * take_off_dis - (active_back_dis + active_back_dis + charge_and_cargo + take_off_dis);
                emergencyScore[i] =  back_dis;
            }
            // init
            else if (car.drones[i]->state == INIT) {
                case_serve = 4;
               emergencyScore[i] = 400;
            }
            else if (car.drones[i]->state == BIND_CAR){
                case_serve = 5;
                emergencyScore[i] = 402;
            }
            // 其他情况，低分
            else {
                case_serve = 6;
                // cout << "test state: " << drone_work_state_str_[int(car.drones[i]->state-1)];
                emergencyScore[i] = 10;
            }
            cout << i << " : " <<  emergencyScore[i] << " case : "<< case_serve << " dis: " << dis << endl;
            
        }
        cout << "--------------------------------------------------------" << endl;
        // 判断emergencyScore中分数最高的，并给出索引
        auto max_score_iter = std::max_element(emergencyScore.begin(), emergencyScore.end());
        if (max_score_iter != emergencyScore.end()) {
            size_t index_of_max_score = std::distance(emergencyScore.begin(), max_score_iter);
            car.serve_index = index_of_max_score;
        } 

        cout << " serve index: " <<  car.serve_index << endl;
        // Drone* drone_ptr = car.drones[0];
        Drone* drone_ptr = car.drones[car.serve_index];
        
        switch (car.state) {
            // 小车在初始的地方，准备去往地勤区
            case WAIT: {  
                car.planning = false;
                car.state = PLAN_LOADING;

                break;
            }

            // 小车规划去地勤区的轨迹
            case PLAN_LOADING: { 
                // 等待规划成功

                if (car.planning) {
                    if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_RUNNING) {
                        car.state = MOVING;
                        car.planning = false;
                    }
                    break;
                }
                if (loading_busy_) {
                    cout << "--Loading busy, waitting" << endl;
                    break;
                }
                // wyc 1021: 去的时候也把订单排序，避免在地勤区等待时间过长（loading cargo排的序过了太久）
                if(!car.waybill_list.empty()){
                    for (size_t i = 0; i < min(10, int(car.waybill_list.size())); ++i) {
                        double score = calculateScore(car.waybill_list[i], car);
                        if (score < 0 && (ros::Time::now().toSec() - car.waybill_list[i]->orderTime / 1000) > 0) {
                            car.waybill_list.erase(car.waybill_list.begin() + i);
                        }
                    }
                }
                car.ordered_waybill_list_ = car.waybill_list;     
                std::sort(car.ordered_waybill_list_.begin(), car.ordered_waybill_list_.end(),
                    [](auto a, auto b) {
                        return a->orderTime < b->orderTime;
                    });
                
                std::sort(car.ordered_waybill_list_.begin(), (car.ordered_waybill_list_.begin() + std::min(10, (int)car.ordered_waybill_list_.size())),
                    [&car](const BillStatus* a, const BillStatus* b) {
                        double scoreA = calculateScore_Load(a, car);
                        double scoreB = calculateScore_Load(b, car);
                        return scoreA > scoreB;
                    }
                );

                // wyc 1020: 如果sever的飞机状态是init，且最优单减去时间动态余量（车去程）大于5s，那么break
                // wyc 1021: 增加条件：落了飞机+无返程要去上货点，且最优单减去时间动态余量（车去程）大于5s，那么break,如果落飞机+有返程则不参与订单时间判断
                bool emergency_planloading = false;
                if(!car.physical_statu.drone_sn.empty()){  
                    for (auto &drone : car.drones){
                        if(drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0){
                            emergency_planloading = true;
                        }
                    }
                }
                
                // wyc 1021: 检查order订单
                // for (size_t i = 0; i < car.ordered_waybill_list_.size(); ++i) {
                //     std::cout << "Element " << i << ": Order Time: " << car.ordered_waybill_list_[i]->orderTime << "ros_time_now"<< ros::Time::now() << ", Index: " << car.ordered_waybill_list_[i]->index << std::endl;
                // }


                if((car.drones[car.serve_index]->state == INIT || (!car.physical_statu.drone_sn.empty() && !emergency_planloading)) && ((car.ordered_waybill_list_[0]->orderTime / 1000) - ros::Time::now().toSec() - transport_go_time) > 5){
                    cout << "best order too long, break!" << endl;
                    break;
                }
                
                // wyc 1008:如果判断余量不足接新飞机，状态切回AT_PARKING_POINT
                bool right_time = true;
                for (auto &drone : car.drones){
                    double dis = (positionToVector2d(drone->physical_statu.pos.position) - 
                                    positionToVector2d(car.park_point)).norm();
                    // wyc 1021: t_height backpath[2]超出范围，改为0
                    double t_height =  abs(drone->back_path[0].z() - drone->physical_statu.pos.position.z);
                    // wyc 1008: 如果oncar的飞机电量少于30，则charge and cargo时间增加换电时间
                    double charge_and_cargo = 10.0 * 10.0;
                    if (drone->state == ON_CAR && drone->physical_statu.remaining_capacity < 30.0) {
                        charge_and_cargo = 20.0 * 10.0;
                    }
                    
                    // 如果有飞机状态是oncar，忽略后续余量判断
                    if(drone->state == ON_CAR){
                        cout<< "drone is on car, ignore the active dis" << endl;
                        right_time = true;
                        if(car.state == AT_PARKING_POINT){
                            cout << "change state back to plan loading" << endl;
                            car.state = PLAN_LOADING;
                        }
                        break;
                    }
                    // wyc 1021: dis_unload用于添加去程判断，如果很近的去程，就不去load了
                    double dis_unload = (positionToVector2d(drone->physical_statu.pos.position) - 
                                         positionToVector2d(car.unloading_point)).norm();
                    //   wyc 1021: 飞机高度到地距离 + 1.5 是算上加减速的估计，可以用准确时间优化；15*10是数了的5米悬停+降落时间
                    double t_height_down = abs(drone->physical_statu.pos.position.z) * 1.5 + 15 * 10;
                    // wyc 1021: 判断条件修改，增加去程过近的判断
                    // wyc 1025: 由于返程在另一架飞机上货前都不会起飞，这里不需要余量了，都可以注释，但是为了排除特殊情况保留返程判断
                    if(drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0 && dis < (active_back_dis + charge_and_cargo  + take_off_dis - t_height * 1.5))
                                // ((drone->state== UNLODING_CARGO || drone->state== PLAN_BACK)  && dis < (active_back_dis + charge_and_cargo + take_off_dis - take_off_dis)) ||
                                // (drone->state == FLYING && drone->physical_statu.bind_cargo_id != 0 && dis < (active_back_dis + charge_and_cargo + take_off_dis + 5 * 10 - dis_unload -25 * 10 -t_height_down))
                                // )
                    {
                        car.state = AT_PARKING_POINT;
                        right_time = false;
                        // wyc 1008: 防止0号机时符合条件，但是1号机ON_CAR状态，导致错误等待
                        continue;
                    } 
                }

                // wyc 1007: 如果本车serve的飞机状态是init，且另有一车是一架返程低于30米，一架不是INIT，那么break
                // wyc 1025: 在判断init的基础之上，添加条件：如果本车serve的飞机状态是on car，且本车serve的另外一架飞机是INIT
                if(car.drones[car.serve_index]->state == INIT ||
                (car.drones[car.serve_index]->state == ON_CAR && car.drones[abs(1 - car.serve_index)]->state == INIT)
                )
                {
                    for (auto &car_p : car_vector_) {
                        // car_p中 drones 中一个飞机是返程且z小于10，另外一架飞机不是init，right_time = false 
                        int other_index = -1;
                        switch(car_p.serve_index) {
                            case 0:
                                other_index = 1;
                                break;
                            case 1:
                                other_index = 0;
                                break;
                            default:
                                break;
                        }
                        // lh 1024 todo: 细化另一架的紧急判断，如果剩下一架飞机的剩余时间
                        // case1
                        // 如果car_p的非serve飞机是在带货：（（送到卸货点时间+下降时间+悬停时间or下降时间+悬停时间）+ 卸货时间 + 起飞时间 + 返程飞的时间 + 返程降落时间）>=（
                        // 目前车car去地勤的时间 + 5s返回parking的停留时间 + car_p 去地勤事件 + 地勤卸飞机时间 + 5s返回parking的停留时间 + 返程时间）
                        // 如果小于，说明连卸飞机都来不及接，那就等他降落再去
                        if (car_p.drones[car_p.serve_index]->state == FLYING && car_p.drones[car_p.serve_index]->physical_statu.pos.position.z > -45.0 ) {
                            if ((car_p.drones[other_index]->state == PLAN_BACK || car_p.drones[other_index]->state == FLYING || car_p.drones[other_index]->state == UNLODING_CARGO) 
                                && car_p.drones[other_index]->physical_statu.bind_cargo_id != 0) {
                                double ft;  // 总飞行时间
                                double t_acc = 10 / 5;  // 加速时间
                                double d_acc = 0.5 * 5 * t_acc * t_acc;  // 无人机加速距离
                                switch(car_p.drones[other_index]->state) {
                                    case PLAN_BACK: {
                                        ft = 0.5 + 25;  // 起飞
                                        for (int b = 0;  b < car_p.drones[other_index]->back_path.size()-1; b++) {
                                            double b_dis = (car_p.drones[other_index]->back_path[b] - car_p.drones[other_index]->back_path[b+1]).norm();
                                            if (b_dis <= d_acc) {
                                                ft += sqrt(2 * b_dis / 5);  // 短距离
                                            } else {
                                                ft += 2 * t_acc;  // 长距离
                                                ft += (b_dis - d_acc) / 10;
                                            }
                                        }
                                        double d_land = abs(car_p.drones[other_index]->back_path[0].z()) - 16.0 - 5.0;
                                        ft += 2 * t_acc;
                                        ft += (d_land-d_acc) / 10;  // 降落
                                        break;
                                    }
                                    case FLYING: {
                                        ft += (positionToVector3d(car_p.drones[car_p.serve_index]->physical_statu.pos.position) - car_p.drones[other_index]->back_path.back()).norm() / 9.0;  // 平飞                                    
                                        double d_land = abs(car_p.drones[other_index]->back_path[0].z()) - 16.0 - 5.0;
                                        ft += 2 * t_acc;
                                        ft += (d_land-d_acc) / 10;  // 降落
                                        break;
                                    }
                                    case UNLODING_CARGO: {
                                        ft = 2.0 + 0.5 + 25;  // 起飞
                                        for (int b = 0;  b < car_p.drones[other_index]->back_path.size()-1; b++) {
                                            double b_dis = (car_p.drones[other_index]->back_path[b] - car_p.drones[other_index]->back_path[b+1]).norm();
                                            if (b_dis <= d_acc) {
                                                ft += sqrt(2 * b_dis / 5);  // 短距离
                                            } else {
                                                ft += 2 * t_acc;  // 长距离
                                                ft += (b_dis - d_acc) / 10;
                                            }
                                        }
                                        double d_land = abs(car_p.drones[other_index]->back_path[0].z()) - 16.0 - 5.0;
                                        ft += 2 * t_acc;
                                        ft += (d_land-d_acc) / 10;  // 降落
                                        break;
                                    }   
                                }
                                double car_now_t = car.transport_go_time_ + car.transport_back_time_ + 5.0 + car_p.transport_go_time_ + 10.0 + 5.0 + car_p.transport_back_time_;
                                if (ft < car_now_t) {
                                    right_time = false;
                                }
                                break;
                            } 
                        }  
                        // case2
                        // 如果没在带货也不是init的话，就在返程，直接让给它
                        if (car_p.drones[other_index]->state == FLYING && car_p.drones[other_index]->physical_statu.bind_cargo_id == 0) {
                            right_time = false;
                            break;
                        }

                        // if (car_p.car_index_ != 5 && car_p.car_index_ != 6 && car_p.drones[car_p.serve_index]->state == PLAN_BACK && car_p.drones[car_p.serve_index]->physical_statu.pos.position.z > -30.0 
                        //     && (car_p.drones[other_index]->state == PLAN_BACK || car_p.drones[other_index]->state == UNLODING_CARGO || (car_p.drones[other_index]->state == FLYING && car_p.drones[other_index]->physical_statu.bind_cargo_id == 0)) ) {
                        //     right_time = false;
                        //     break;
                        // }                     
                    }
                }
                if(!right_time){
                    cout << car.car_index_<< "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl;
                    cout << "--Other Car need the loading point" << endl; 
                    break;
                }

                // 进行规划
                UserCmdRequest msg;
                msg.peer_id = config_.getPeerId();
                msg.task_guid = config_.getGuid();
                msg.type = UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
                msg.car_route_info.carSn = car.sn;
                msg.car_route_info.yaw = 0.0;
                    

                // if (car.first) {
                //     // wyc 1020: 这里不能直接替换，搜A*
                //     // msg.car_route_info.way_point[0] = car.physical_statu.pos.position;
                //     // car.first = false;
                // }
                if(car.first){
                    vector<Vector3d> temp_init_path;
                    vector<Vector3d> first_path;
                    vector<Vector3d> nouse;
                    cout<< car.physical_statu.pos.position << endl;
                    Vector3d start = positionToVector3d(car.physical_statu.pos.position)  - Vector3d(0, 0, 1.1);
                    Vector3d end = positionToVector3d(loading_cargo_point_) - Vector3d(0, 0, 1.1);
                    if (judgeCarLine(start, end, 0.5)) {
                            // 判断直线是否可行
                            first_path.push_back(start);
                            first_path.push_back(end);
                            cout << "-Car init line found!" << endl;
                        } else {
                            // A*
                            int result = astar_.aStarSearch(map_handle_, start, end, 0.5, 2.1, temp_init_path, nouse);
                            if (result == Astar::REACH_END) {
                                shortCarPath(temp_init_path,first_path);
                            } else if (result == Astar::NO_PATH) { 
                                cout << "No car path found." << endl;
                                double dis_order = (positionToVector2d(car.physical_statu.pos.position) - positionToVector2d(car.park_point)).norm();
                                if(dis_order<2.5){
                                    car.first = false;
                                }

                                break;
                            }
                        }
                        for (auto &car_p : first_path) {
                        Position p;
                        p.x = car_p.x();
                        p.y = car_p.y();
                        p.z = car_p.z();
                        msg.car_route_info.way_point.push_back(p);
                        }
                }else{
                    for (auto &car_p : car.load_path) {
                        Position p;
                        p.x = car_p.x();
                        p.y = car_p.y();
                        p.z = car_p.z();
                        msg.car_route_info.way_point.push_back(p);
                    }
                }

                // wyc 1020: 发布轨迹必须在车状态为ready时
                if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_READY){
                    cmd_pub_.publish(msg);  
                    load_flag_ = true;
                    car.planning = true;
                    loading_busy_ = true;
                }
                cout << "---Publishing first car plan msg and busy true:" << " car sn: " << msg.car_route_info.carSn
                     << " car route info: " << msg.car_route_info;
                // std::cout << "Press any key to continue..." << std::endl;
                // std::cin.get();
                break;
            }

            // 小车运动中
            case MOVING: {
                car.planning = false;
                if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_READY) {
                    double dis = (positionToVector3d(car.physical_statu.pos.position) - 
                                positionToVector3d(loading_cargo_point_)).norm();
                    if (dis > 0.5) {
                        // cout << "---" << car.sn << " arrive parking point"<< endl;
                        car.state = AT_PARKING_POINT;
                    } else {
                        // cout << "---" << car.sn << " arrive loading point"<< endl;
                        car.state = AT_LOADING_POINT;
                    }
                }
                break;
            }

            // 小车在地勤区，（上飞机）进行装货和换电
            case AT_LOADING_POINT: {    
                car.planning = false;        
                if (car.physical_statu.drone_sn.empty()) {
                    // 如果有飞机马上回来了 OR 下货/规划返程点特别近，不要绑定飞机，安排车返回航空点
                    bool right_time = true;
                    for (auto &drone : car.drones){
                        // wyc 0930: 调整绑车条件
                        // wyc 1008: 再次调整
                        double dis_xy = (positionToVector2d(drone->physical_statu.pos.position) - positionToVector2d(car.park_point)).norm();  // 返程距离
                        double dis_z = abs(drone->physical_statu.pos.position.z - car.park_point.z);  // 高度差
                        double dis = dis_xy + dis_z;
                        // wyc 1021: t_height backpath[2]超出范围，改为0
                        double t_height =  abs(drone->back_path[0].z() - drone->physical_statu.pos.position.z);
                        // wyc 1008: 如果往回飞的飞机的电量小于45 and cargo时间增加换电时间
                        double charge_and_cargo = 10.0 * 10.0;
                        if ((drone->physical_statu.remaining_capacity - (dis_xy / 10)) < 40.0) {
                            charge_and_cargo = 20.0 * 10.0;
                        }
                    // wyc 1025: 由于添加返程起飞条件，这里不需要余量了
                    if(drone->state == FLYING && drone->physical_statu.bind_cargo_id == 0 && dis < (active_back_dis + active_go_dis + charge_and_cargo  + take_off_dis - t_height))
                        // ((drone->state== UNLODING_CARGO || drone->state== PLAN_BACK)  && dis < (active_back_dis + active_go_dis + charge_and_cargo + take_off_dis - take_off_dis))
                        // )
                            {
                            right_time = false;
                            car.state = PLAN_PARKING;
                            break;
                        }
                    }
                    if(!right_time){
                        // cout << "---not right time for drone binding: car back~" << endl;
                        break;
                    } 
                    // 小车空空，上飞机
                    // cout << "---Waitting bind drone~" << endl;
                    // wyc 1008: debug
                    if(drone_ptr->state == INIT){
                        drone_ptr->state = BIND_CAR;
                    }
                } else {
                    // cout << "---Waitting drone loading cargo and charge" << endl;
                    // wyc 1025 debug：如果飞机是oncar，才给loading_cargo
                    cout << "---Waitting drone loading cargo and charge" << endl;
                    if (drone_ptr->physical_statu.bind_cargo_id == 0 && drone_ptr->state == ON_CAR) {
                        drone_ptr->state = LOADING_CARGO;
                    }
                }
                break;
            }

            // 小车规划去park point的轨迹
            case PLAN_PARKING: {
                // 等待规划成功
                if (car.planning) {
                    if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_RUNNING) {
                        car.state = MOVING;
                        car.planning = false;
                        load_flag_ = false;
                    }  
                    break;
                }
                // if (car.physical_statu.car_work_state == CarPhysicalStatus::CAR_RUNNING) break;

                // 进行规划
                UserCmdRequest msg;
                msg.peer_id = config_.getPeerId();
                msg.task_guid = config_.getGuid();
                msg.type = UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
                msg.car_route_info.carSn = car.sn;
                msg.car_route_info.yaw = 0.0;
                for (auto &car_p : car.park_path) {
                    Position p;
                    p.x = car_p.x();
                    p.y = car_p.y();
                    p.z = car_p.z();
                    msg.car_route_info.way_point.push_back(p);
                }
                // wyc 1020:小车状态为ready,server的飞机ready才规划 或者是返程的飞机需要
                if(car.physical_statu.car_work_state == CarPhysicalStatus::CAR_READY && ( car.drones[car.serve_index]->physical_statu.drone_work_state == DronePhysicalStatus::READY || car.drones[car.serve_index]->physical_statu.drone_work_state == DronePhysicalStatus::FLYING)){
                    planback_count++;
                    if(planback_count > 5){
                        cout << "---Publishing park car plan msg:" << " car sn: " << msg.car_route_info.carSn << " car route info: " << msg.car_route_info;
                        cmd_pub_.publish(msg);
                        car.planning = true;
                        planback_count = 0;
                    }

                }


                break;

            }

            // 小车在起飞点，飞机起飞/接机
            case AT_PARKING_POINT: {
                car.planning = false;
                if (car.physical_statu.drone_sn.empty()) {
                    // 如果在parking point，而且上一架飞机已经飞很远了，可以安排去loading point
                    
                    // cout << "---Waitting drone landing" << endl;
                } else {
                    if (!drone_ptr->physical_statu.bind_cargo_id == 0) {
                        // cout << "---Waitting drone flying" << endl;
                        drone_ptr->state = PLAN_GO;
                    } else {
                        // cout << "---taking drone back" << endl;
                    }
                }
                break;
            }
        }        
    }
}  


// ============================= normal function =============================
/**
 * @brief 初始化
*/
bool initMtuav() {
    cout << "****************************************************************************************" << endl;
    cout << "************************************** init start **************************************" << endl;;
    if (info_sub_.getTopic().empty() && cmd_resp_sub_.getTopic().empty()) {
        cout << "Failed to subscribe to /panoramic_info topic or /cmd_resp topic" << endl;
    } else {
        cout << "Successfully subscribed to /panoramic_info topic and /cmd_resp topic" << endl;
    }

    // read json
    ifstream jsonFile("/config/config.json", ifstream::binary);
    if (!jsonFile.is_open()) {
        cout << "---Unable to open json file" << endl;
        return false;
    }
    cout << "---Successfully open user config file" << endl;
    Json::Value root;
    jsonFile >> root;
    config_.LoadFromJson(root);

    // read 地勤点
    loading_busy_ = false;
    loading_cargo_point_.x = config_.getLoadingCargoPoint().x;
    loading_cargo_point_.y = config_.getLoadingCargoPoint().y;
    loading_cargo_point_.z = config_.getLoadingCargoPoint().z;
    cout << "---loading cargo point: (" << loading_cargo_point_.x << ", " 
                                    << loading_cargo_point_.y << ", " 
                                    << loading_cargo_point_.z << ")" << endl;

    // read 机/车信息
    load_flag_ = false;
    cout << "---Read drone param: " << endl;
    for (auto drone_param : config_.getDroneParamList()) {
        Drone drone;
        drone.planning = false;
        drone.first = true;
        drone.sn = drone_param.droneSn;
        drone.state = DroneWorkState::INIT;
        cout << drone.sn << " ";
        drone_vector_.push_back(drone);
    }
    cout << endl;
    cout << "---Read car param: " << endl;
    for (auto car_param : config_.getCarParamList()) { 
        Car car;
        car.planning = false;
        car.first = true;
        car.sn = car_param.magvSn;
        car.state = CarWorkState::WAIT;
        car.waybill_count = 0;
        car.init_pos_.x = car_param.birthplace.x;
        car.init_pos_.y = car_param.birthplace.y;
        car.init_pos_.z = car_param.birthplace.z;
        // cout << car.sn << " ";
        car.serve_index = 0;
        car.closest_back_drone_dis = 1000000000;
        car_vector_.push_back(car);
    }
    cout << endl;
    
    // 只保留4辆车，按照车初始位置到(190, 425)距离选择
    std::sort(car_vector_.begin(), car_vector_.end(), [&loading_cargo_point_](const Car& a, const Car& b) {
        return (positionToVector2d(a.init_pos_)-positionToVector2d(loading_cargo_point_)).norm() < (positionToVector2d(b.init_pos_)-positionToVector2d(loading_cargo_point_)).norm();
    });
    if (car_vector_.size() > 4) {
        car_vector_.resize(4);
    }
    // 输出保留的车辆编号
    for (const auto& car : car_vector_) {
        std::cout << car.sn << " ";
    }
    std::cout << std::endl;

    // 分配车机组
    size_t num_cars = car_vector_.size();
    size_t num_drones = drone_vector_.size();
    int car_index = 0;
    for (int i = 0; i < num_drones; i++) {
        if (car_vector_[car_index].drones.size() < 2) {
            car_vector_[car_index++].drones.push_back(&drone_vector_[i]);
        }
        if (car_index >= num_cars) car_index = 0;
    }
    // 打印分配结果
    for (const auto& car : car_vector_) {
        std::cout << "--Car " << car.sn << " has " << car.drones.size() << " drones." << std::endl;
        for (const auto& drone_ptr : car.drones) {
            std::cout << " -Drone SN: " << drone_ptr->sn << std::endl;
        }
    }

    // read
    birth_pos_.x = config_.getDroneParamList()[0].birthplace.x;
    birth_pos_.y = config_.getDroneParamList()[0].birthplace.y;
    birth_pos_.z = config_.getDroneParamList()[0].birthplace.z;

    // read 卸货点
    cout << "---unloading cargo point:" << endl;
    vector<Position> unloading_points;
    int i = 0;
    for (auto &unloading_point_param : config_.getUnloadingCargoStationList()) {
        Position ulp;
        ulp.x = unloading_point_param.position.x;
        ulp.y = unloading_point_param.position.y;
        ulp.z = unloading_point_param.position.z;
        unloading_points.push_back(ulp);
    }
    vector<bool> is_point_taken(car_vector_.size(), false);
    for (auto &up : unloading_points) {
        double min_distance = std::numeric_limits<double>::max();
        int closest_car_index = -1;
        for (int i = 0; i < car_vector_.size(); ++i) {
            if (!is_point_taken[i]) {
                double dist = (positionToVector3d(car_vector_[i].init_pos_) - positionToVector3d(up)).norm();
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_car_index = i;
                }
            }
        }
        if (closest_car_index != -1) {
            car_vector_[closest_car_index].unloading_point = up;
            cout << "--" << car_vector_[closest_car_index].sn << ": (" 
                 << car_vector_[closest_car_index].unloading_point.x << ", " 
                 << car_vector_[closest_car_index].unloading_point.y << ", " 
                 << car_vector_[closest_car_index].unloading_point.z << ")" << endl;
            is_point_taken[closest_car_index] = true;
        }
    }

    // read 机/车边界
    drone_map_min_.x = config_.getMapBoundaryInfo().bottomLeft.x;
    drone_map_min_.y = config_.getMapBoundaryInfo().bottomLeft.y;
    drone_map_min_.z = -60.0;
    drone_map_max_.x = config_.getMapBoundaryInfo().topRight.x;
    drone_map_max_.y = config_.getMapBoundaryInfo().topRight.y;
    drone_map_max_.z = -120.0;
    car_map_min_.x = config_.getMagvAviationOperationsBoundaryInfo().bottomLeft.x;
    car_map_min_.y = config_.getMagvAviationOperationsBoundaryInfo().bottomLeft.y;
    car_map_min_.z = config_.getMagvAviationOperationsBoundaryInfo().bottomLeft.z;
    car_map_max_.x = config_.getMagvAviationOperationsBoundaryInfo().topRight.x;
    car_map_max_.y = config_.getMagvAviationOperationsBoundaryInfo().topRight.y;
    car_map_max_.z = config_.getMagvAviationOperationsBoundaryInfo().topRight.z;
    cout << "---drone boundary: (" << drone_map_min_.x << ", " << drone_map_min_.y << ", " << drone_map_min_.z << ") -> ("
                                << drone_map_max_.x << ", " << drone_map_max_.y << ", " << drone_map_max_.z << ")" << endl;
    cout << "---car boundary: (" << car_map_min_.x << ", " << car_map_min_.y << ", " << car_map_min_.z << ") -> ("
                              << car_map_max_.x << ", " << car_map_max_.y << ", " << car_map_max_.z << ")" << endl;

    // map handle init
    std::string localMapPath = "/home/sdk_for_user/map_client_sdk/for_cpp/voxel_map_final.bin";
    map_handle_ = MtMapCreateLocal(const_cast<char*>(localMapPath.c_str()));
    if (map_handle_ == 0) {
        std::cerr << "---Failed to create local map client" << std::endl;
        return false;
    }

    // cout << "************************************** test start **************************************" << endl; 
    // mtVoxel voxel;
    // if (MtMapQuery(map_handle_, 498, 528, -2, &voxel) == 0) {
    //     cout << "Failed to query voxel" << endl;
    //     return false;
    // }else{
    //     cout << voxel.height_to_ground << " " << voxel.distance << endl;
    // }
    // cout << "************************************** test end **************************************" << endl;

    // waybill
    new_waybill_set_ = false;
    cout << "*************************************** init end ***************************************" << endl;
    cout << "****************************************************************************************" << endl;

    // init car/drone path
    cout << "*********************************************************************************************************" << endl;
    cout << "************************************** search car/drone path start **************************************" << endl;
    
    if (!searchCarPathMap()) {
        return false;
    } else {
        cout << "****************************************************************************************" << endl;
        if (!searchDronePathMap()) {
            return false;
        }
    }

    cout << "Please wait for 30s to initialize node" << endl;
    ros::Duration(30).sleep();

    // car run
    for (auto &car : car_vector_) {
        UserCmdRequest msg;
        msg.peer_id = config_.getPeerId();
        msg.task_guid = config_.getGuid();
        msg.type = UserCmdRequest::USER_CMD_CAR_EXEC_ROUTE;
        msg.car_route_info.carSn = car.sn;
        msg.car_route_info.yaw = 0.0;

        // search init path
        vector<Vector3d> temp_init_path;
        vector<Vector3d> final_init_path;
        vector<Vector3d> nouse;
        Vector3d start = positionToVector3d(car.init_pos_)  - Vector3d(0, 0, 1.1);
        Vector3d end = positionToVector3d(car.park_point) - Vector3d(0, 0, 1.1);
        cout << "start: " << start << " end: " << end << endl;
        if (judgeCarLine(start, end, 0.5)) {
            // 判断直线是否可行
            final_init_path.push_back(start);
            final_init_path.push_back(end);
            cout << "-Car init line found!" << endl;
        } else {
            // A*
            int result = astar_.aStarSearch(map_handle_, start, end, 0.5, 2.1, temp_init_path, nouse);
            if (result == Astar::REACH_END) {
                shortCarPath(temp_init_path, final_init_path);
                cout << "-Car A* path " << car.car_index_ << " found: ";
                for (const Vector3d& p : final_init_path) {
                    cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
                }
                cout << endl;
            } else if (result == Astar::NO_PATH) { 
                cout << "No car path found." << endl;
                return false;
            }
        }
        
        for (const Vector3d& p : final_init_path) {
            Position pos;
            pos.x = p.x();
            pos.y = p.y();
            pos.z = car.init_pos_.z;
            msg.car_route_info.way_point.push_back(pos);
        }
        cmd_pub_.publish(msg);
        // cout << "testtttttttttttttttttttttttttt" << endl;
        cout << "---Publishing car plan msg:" << " car sn: " << msg.car_route_info.carSn << " car route info: " << msg.car_route_info;
    }
    cout << "*********************************** search car/drone path end **************************" << endl;
    cout << "****************************************************************************************" << endl;


    return true;
}

/**
 * @brief 查找car索引
 * @param sn 车sn
 * @return car_physical_status的索引值
*/
int findCarIndexBySN(const vector<CarPhysicalStatus>& car_physical_status, const string& sn) {
    auto it = find_if(car_physical_status.begin(), car_physical_status.end(),
                      [&sn](const CarPhysicalStatus& status) { return status.sn == sn; });

    if (it != car_physical_status.end()) {
        return distance(car_physical_status.begin(), it);
    } else {
        cout << "findCarIndexBySN: " << sn << " not found" << endl;
        return -1; 
    }
}

/**
 * @brief 查找drone索引
 * @param sn sn
 * @return drone_physical_status的索引值
*/
int findDroneIndexBySN(const vector<DronePhysicalStatus>& drone_physical_status, const string& sn) {
    auto it = find_if(drone_physical_status.begin(), drone_physical_status.end(),
                           [&sn](const DronePhysicalStatus& status) { return status.sn == sn; });

    if (it != drone_physical_status.end()) {
        return distance(drone_physical_status.begin(), it);
    } else {
        cout << "findDroneIndexBySN: " << sn << " not found" << endl;
        return -1;
    }
}

/**
 * @brief position -> Vector3d
*/
Vector3d positionToVector3d(const Position& p) {
    return Vector3d(p.x, p.y, p.z);
}
Vector2d positionToVector2d(const Position& p) {
    return Vector2d(p.x, p.y);
}

/**
 * @brief Vector3d -> position
*/
Position vector3dToPosition(const Vector3d& vp) {
    Position p;
    p.x = vp.x();
    p.y = vp.y();
    p.z = vp.z();
    return p;
}


// ============================= plan function =============================
/**
 * @brief 初始化车辆的路线图
*/
bool searchCarPathMap() {
    double radius = 7.1;
    int car_num = car_vector_.size();

    // set car parking spots
    vector<Position> park_points;
    park_points.resize(car_num);
    Position park_point;
    park_point.x = 186;
    park_point.y = 431;
    park_point.z = car_map_min_.z;
    park_points[0] = park_point;
    park_point.x = 194;
    park_point.y = 431;
    park_points[1] = park_point;
    park_point.x = 181;
    park_point.y = 434;
    park_points[2] = park_point;
    park_point.x = 199;
    park_point.y = 434;
    park_points[3] = park_point;
    // double mid_x = loading_cargo_point_.x;
    // double mid_y = car_map_min_.y + 1.1; 
    // for (int i = 0; i < ceil(car_num / 2); i++) {
    //     park_point.x = mid_x - 4.1;
    //     park_point.y = mid_y;
    //     park_point.z = car_map_min_.z;
    //     park_points[2*i] = park_point;

    //     if (2*i+1 < car_vector_.size()) {
    //         park_point.x = mid_x + 4.1;
    //         park_point.y = mid_y;
    //         park_point.z = car_map_min_.z;
    //         park_points[2*i+1] = park_point;
    //     }
    //     mid_y += radius;
    // }

    // 分配停车点
    int car_index = 1;
    vector<bool> is_car_taken(car_vector_.size(), false);
    for (auto &park : park_points) {
        double min_distance = 100000;
        int closest_car_index = -1;

        for (int i = 0; i < car_vector_.size(); ++i) {
            if (!is_car_taken[i]) {
                double dist = (positionToVector3d(car_vector_[i].init_pos_) - positionToVector3d(park)).norm();
                if (dist < min_distance) {
                    min_distance = dist;
                    closest_car_index = i;
                }
            }
        }
        if (closest_car_index != -1) {
            car_vector_[closest_car_index].park_point = park;
            car_vector_[closest_car_index].car_index_ = car_index;
            car_index += 1;
            is_car_taken[closest_car_index] = true;
            cout << "-" << car_vector_[closest_car_index].sn << " parking point: (" 
                 << car_vector_[closest_car_index].park_point.x << ", " 
                 << car_vector_[closest_car_index].park_point.y << ", " 
                 << car_vector_[closest_car_index].park_point.z << ") " 
                 << "car index: " << car_vector_[closest_car_index].car_index_
                 << endl;
        }
    }
    
    // wyc 0926: 5号车和6号车只有一个飞机，所以直接移除一个飞机
    // for (auto &car : car_vector_) {
    //     if (car.car_index_ == 5 || car.car_index_ == 6) {
    //         if (!car.drones.empty()) { // 确保容器不为空
    //             car.drones.pop_back(); // 直接移除最后一个元素
    //         }
    //     }
    // }

    // wyc 1024: 修改最近那那两辆车的路径，避免他们碰撞
    // set path
    for (auto& car : car_vector_){
        switch(car.car_index_) {
            case 1:
                car.load_path.push_back(positionToVector3d(car.park_point));
                // wyc 1024: 修改路径
                car.load_path.push_back(Vector3d(188, 425, car_map_min_.z));
                car.load_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path = car.load_path;
                reverse(car.park_path.begin(), car.park_path.end());
                break;
            case 2:
                car.load_path.push_back(positionToVector3d(car.park_point));
                // wyc 1024: 修改路径
                car.load_path.push_back(Vector3d(192, 425, car_map_min_.z));
                car.load_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path = car.load_path;
                reverse(car.park_path.begin(), car.park_path.end());
                break;
            case 3:
                car.load_path.push_back(positionToVector3d(car.park_point));
                car.load_path.push_back(Vector3d(181, 425, car_map_min_.z));
                car.load_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path.push_back(Vector3d(190, 422, car_map_min_.z));
                car.park_path.push_back(Vector3d(181, 422, car_map_min_.z));
                car.park_path.push_back(positionToVector3d(car.park_point));
                break;
            case 4:
                car.load_path.push_back(positionToVector3d(car.park_point));
                car.load_path.push_back(Vector3d(199, 425, car_map_min_.z));
                car.load_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path.push_back(Vector3d(190, 425, car_map_min_.z));
                car.park_path.push_back(Vector3d(190, 422, car_map_min_.z));
                car.park_path.push_back(Vector3d(199, 422, car_map_min_.z));
                car.park_path.push_back(positionToVector3d(car.park_point));
                break;
        }
    }
    
    // for (int i = 0; i < car_num; i++) {
    //     double dis = abs(car_vector_[i].park_point.y - car_map_min_.y);
    //     if (dis < radius) {
    //         car_vector_[i].load_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //         car_vector_[i].load_path.push_back(positionToVector3d(loading_cargo_point_));
    //         car_vector_[i].park_path.push_back(positionToVector3d(loading_cargo_point_));
    //         Vector3d p_mid = Vector3d(car_vector_[i].park_point.x, loading_cargo_point_.y, loading_cargo_point_.z);
    //         car_vector_[i].park_path.push_back(p_mid);
    //         car_vector_[i].park_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //     } else {
    //         if (car_vector_[i].park_point.x < loading_cargo_point_.x) {
    //             car_vector_[i].load_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //             car_vector_[i].load_path.push_back(Vector3d(loading_cargo_point_.x, car_vector_[i].park_point.y, car_map_min_.z));
    //             car_vector_[i].load_path.push_back(positionToVector3d(loading_cargo_point_));
    //             car_vector_[i].park_path.push_back(positionToVector3d(loading_cargo_point_));
    //             car_vector_[i].park_path.push_back(Vector3d(car_vector_[i].park_point.x-4.0, loading_cargo_point_.y, loading_cargo_point_.z));
    //             car_vector_[i].park_path.push_back(Vector3d(car_vector_[i].park_point.x-4.0, car_vector_[i].park_point.y, loading_cargo_point_.z));
    //             car_vector_[i].park_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //         } else {
    //             car_vector_[i].load_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //             car_vector_[i].load_path.push_back(Vector3d(loading_cargo_point_.x, car_vector_[i].park_point.y, car_map_min_.z));
    //             car_vector_[i].load_path.push_back(positionToVector3d(loading_cargo_point_));
    //             car_vector_[i].park_path.push_back(positionToVector3d(loading_cargo_point_));
    //             car_vector_[i].park_path.push_back(Vector3d(car_vector_[i].park_point.x+4.0, loading_cargo_point_.y, loading_cargo_point_.z));
    //             car_vector_[i].park_path.push_back(Vector3d(car_vector_[i].park_point.x+4.0, car_vector_[i].park_point.y, loading_cargo_point_.z));
    //             car_vector_[i].park_path.push_back(positionToVector3d(car_vector_[i].park_point));
    //         }
    //     }
    // }

    for (int i = 0; i < car_num; i++) {
        cout << "-" << car_vector_[i].sn << "load path: " << endl;
        for (const Vector3d& p : car_vector_[i].load_path) {
            cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
        }
        cout << endl;
        cout << "-" << car_vector_[i].sn << "park path: " << endl;
        for (const Vector3d& p : car_vector_[i].park_path) {
            cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
        }
        cout << endl;
    }
    
    return true;
}

/**
 * @brief 初始化飞机的路线图
*/
bool searchDronePathMap() {
    mtVoxel voxel;
    int flight_flag = 0;
    for (int i = 0; i < car_vector_.size(); i++) {
        vector<Vector3d> drone_go_path;
        vector<Vector3d> drone_back_path;
        vector<Vector3d> temp_drone_go_path;
        vector<Vector3d> temp_drone_back_path;

        // 分配高度
        double z_go, z_back;
        z_go = flight_z_pair_[i].first;
        z_back = flight_z_pair_[i].second;

        cout << car_vector_[i].sn << ": z_go: " << z_go << " z_back: " << z_back << endl;
        
        // search drone path
        Vector3d go_s = Vector3d(car_vector_[i].park_point.x, car_vector_[i].park_point.y, z_go);
        Vector3d go_e = Vector3d(car_vector_[i].unloading_point.x, car_vector_[i].unloading_point.y, z_go);
        Vector3d back_s = Vector3d(car_vector_[i].unloading_point.x, car_vector_[i].unloading_point.y, z_back);
        Vector3d back_e = Vector3d(car_vector_[i].park_point.x, car_vector_[i].park_point.y, z_back);

        // cout << "go_s" << go_s.transpose() << " go_e" << go_e.transpose() << " back_s" << back_s.transpose() << " back_e" << back_e.transpose() << endl;
        
        
        vector<Vector3d> other_car_poses;
        for (int k = 0; k < car_vector_.size(); k++) {
            if (k == i) continue;
            Position cp = car_vector_[k].park_point;
            cp.z = z_go;
            other_car_poses.push_back(positionToVector3d(cp));
            // cout << "other car pose: " << other_car_poses.back().transpose() << endl;
        }
        // go path
        if (judgeLine(go_s, go_e, 2.1, other_car_poses)) {
            // 判断直线是否可行
            drone_go_path.push_back(go_s);
            drone_go_path.push_back(go_e);
            cout << "-Drone go line path " << i+1 << " found: ";
            cout << "(" << drone_go_path[0].x() << ", " << drone_go_path[0].y() << ", " << drone_go_path[0].z() << ") "
                 << "(" << drone_go_path[1].x() << ", " << drone_go_path[1].y() << ", " << drone_go_path[1].z() << ")" << endl;
        } else {
            // A*
            int result = astar_.aStarSearch(map_handle_, go_s, go_e, 2.1, obstacle_clearance, temp_drone_go_path, other_car_poses);
            if (result == Astar::REACH_END) {
                shortPath(temp_drone_go_path, drone_go_path, other_car_poses);
                cout << "-Drone go A* path " << i+1 << " found: ";
                for (const Vector3d& p : drone_go_path) {
                    cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
                }
                cout << endl;
            } else if (result == Astar::NO_PATH) { 
                cout << "No path found." << endl;
                return false;
            }
        }

        for (int k = 0; k < other_car_poses.size(); k++) {
            other_car_poses[k][2] = z_back;
        }
        // back path
        if (judgeLine(back_s, back_e, 2.1, other_car_poses)) {
            // 判断直线是否可行
            drone_back_path.push_back(back_s);
            drone_back_path.push_back(back_e);
            cout << "-Drone back line path " << i+1 << " found: ";
            cout << "(" << drone_back_path[0].x() << ", " << drone_back_path[0].y() << ", " << drone_back_path[0].z() << ") "
                 << "(" << drone_back_path[1].x() << ", " << drone_back_path[1].y() << ", " << drone_back_path[1].z() << ")" << endl;
        } else {
            // A*
            int result = astar_.aStarSearch(map_handle_, back_s, back_e, 2.1, obstacle_clearance, temp_drone_go_path, other_car_poses);
            if (result == Astar::REACH_END) {
                shortPath(temp_drone_go_path, drone_back_path, other_car_poses);
                cout << "-Drone back A* path " << i+1 << " found: ";
                for (const Vector3d& p : drone_back_path) {
                    cout << "(" << p.x() << ", " << p.y() << ", " << p.z() << ") ";
                }
                cout << endl;
            } else if (result == Astar::NO_PATH) {
                cout << "No path found." << endl;
                return false;
            }
        }

        for (auto &drone : car_vector_[i].drones) {
            drone->go_path = drone_go_path;
            drone->back_path = drone_back_path;
        }
    }

    return true;
}

/**
 * @brief 判断两点连线是否可行(Position)
*/
bool judgeLine(const Position& pos1, const Position& pos2, double resolution, vector<Vector3d> &other_car_poses) {
    Vector3d vp1 = positionToVector3d(pos1);
    Vector3d vp2 = positionToVector3d(pos2);
    return judgeLine(vp1, vp2, resolution, other_car_poses);
}

/**
 * @brief 判断两点连线是否可行(Vector3d)
*/
bool judgeLine(const Vector3d& pos1, const Vector3d& pos2, double resolution, vector<Vector3d> &other_car_poses) {
    mtVoxel voxel;
    Vector3d dir = pos1 - pos2;
    double len = dir.norm();
    dir.normalize();
    for (double l = resolution; l <= len + 1e-1; l += resolution) {
        Vector3d ckpt = pos1 - l * dir;
        if (MtMapQuery(map_handle_, ckpt.x(), ckpt.y(), ckpt.z(), &voxel) == 0) {
            cout << "Failed to query voxel" << endl;
            return false;
        }
        if (voxel.distance <= 4.1) {
            return false;
        }
        for (auto& oc : other_car_poses) {
            if ((ckpt - oc).norm() < 6.1) {
                // cout << "---line car line failed" << endl;
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief 判断两点连线是否可行(Position)
*/
bool judgeCarLine(const Position& pos1, const Position& pos2, double resolution) {
    Vector3d vp1 = positionToVector3d(pos1);
    Vector3d vp2 = positionToVector3d(pos2);
    return judgeCarLine(vp1, vp2, resolution);
}

/**
 * @brief 判断两点连线是否可行(Vector3d)
*/
bool judgeCarLine(const Vector3d& pos1, const Vector3d& pos2, double resolution) {
    mtVoxel voxel;
    Vector3d dir = pos1 - pos2;
    double len = dir.norm();
    dir.normalize();
    for (double l = resolution; l <= len + 1e-1; l += resolution) {
        Vector3d ckpt = pos1 - l * dir;
        if (MtMapQuery(map_handle_, ckpt.x(), ckpt.y(), ckpt.z(), &voxel) == 0) {
            cout << "Failed to query voxel" << endl;
            return false;
        }
        double voxel_new = voxel.distance;
        // wyc 1019 修改错误体素距离
        if (ckpt.z() > -18.1 &&
            ckpt.y() >= 430.1 && ckpt.y() <= 449.9 &&
            ckpt.x() >= 180.1 && ckpt.x() <= 199.9 &&
            !((ckpt.x() > 185.2 && ckpt.x() < 196.8) && (ckpt.y() > 431.2 && ckpt.y() < 434.8)) &&
            !((ckpt.x() > 183.2 && ckpt.x() < 188.8) && (ckpt.y() > 439.2 && ckpt.y() < 444.8)) &&
            !((ckpt.x() > 193.2 && ckpt.x() < 198.8) && (ckpt.y() > 439.2 && ckpt.y() < 444.8))
            ) {
            // 如果条件满足，修改 voxel.distance 的值为 4
            voxel_new = 4.0;
        }


        cout << "x " << ckpt.x() << " y " << ckpt.y() << " z " << ckpt.z() << " distance " << voxel_new << endl;
        if (voxel_new < 2.1) {
            return false;
        }
    }
    return true;
}

/**
 * @brief 判断两线是否相交
*/
bool judgeLineCross(const Vector2d& l1p1, const Vector2d& l1p2, const Vector2d& l2p1, const Vector2d& l2p2) {
    auto minMax = [](const Vector2d& p1, const Vector2d& p2) {
        return std::make_pair(std::min(p1.x(), p2.x()), std::max(p1.x(), p2.x()));
    };

    auto [minX1, maxX1] = minMax(l1p1, l1p2);
    auto [minY1, maxY1] = minMax(l1p1, l1p2);
    auto [minX2, maxX2] = minMax(l2p1, l2p2);
    auto [minY2, maxY2] = minMax(l2p1, l2p2);

    if (maxX1 < minX2 || minX1 > maxX2 || maxY1 < minY2 || minY1 > maxY2) {
        return false;
    }

    // 计算两线段的参数t1和t2
    auto crossProduct = [](const Vector2d& a, const Vector2d& b) {
        return a.x() * b.y() - a.y() * b.x();
    };

    double d1 = crossProduct(l1p2 - l1p1, l2p1 - l1p1);
    double d2 = crossProduct(l1p2 - l1p1, l2p2 - l1p1);
    double d3 = crossProduct(l2p2 - l2p1, l1p1 - l2p1);
    double d4 = crossProduct(l2p2 - l2p1, l1p2 - l2p1);

    if ((d1 * d2 > 0) || (d3 * d4 > 0)) {
        return false;
    }

    return true;
}

/**
 * @brief 缩短A*路径
*/
void shortPath(const vector<Vector3d>& temp_path, vector<Vector3d>& short_path, vector<Vector3d> &other_car_poses) {
    if (temp_path.empty()) {
        ROS_ERROR("-Empty path to shorten");
        return;
    } else {
        cout << "-Begin short path" << endl;
    }

    short_path.push_back(temp_path[0]);
    for (int i = 1; i < temp_path.size() - 1; ++i) {
        if (!judgeLine(short_path.back(), temp_path[i+1], 2.1, other_car_poses)) {
            short_path.push_back(temp_path[i]);
        }
    }

    if ((temp_path.back() - short_path.back()).norm() > 0.5) {
        short_path.push_back(temp_path.back());
    }
}
void shortCarPath(const vector<Vector3d>& temp_path, vector<Vector3d>& short_path) {
    if (temp_path.empty()) {
        ROS_ERROR("-Empty path to shorten");
        return;
    } else {
        cout << "-Begin short car path" << endl;
    }

    short_path.push_back(temp_path[0]);
    for (int i = 1; i < temp_path.size() - 1; ++i) {
        if (!judgeCarLine(short_path.back(), temp_path[i+1], 2.1)) {
            short_path.push_back(temp_path[i]);
        }
    }

    if ((temp_path.back() - short_path.back()).norm() > 0.5) {
        short_path.push_back(temp_path.back());
    }
}



// ============================= ros =============================
/**
 * @brief 全局信息回调
*/
void panoramicInfoCallback(const PanoramicInfo::ConstPtr& msg) {
    recevie_panoramic_info_ = true;
    current_panoramic_info_ = msg;
    waybill_list_ = msg->bills;

    if (!new_waybill_set_) {
        for(auto &wb : waybill_list_) {
            for (int i = 0; i < car_vector_.size(); i++) {
                // cout << positionToVector3d(wb.target_pos).transpose() << endl;
                // cout << positionToVector3d(car_vector_[i].unloading_point).transpose() << endl;
                // cout << endl;
                double dis = (positionToVector3d(wb.target_pos) - 
                              positionToVector3d(car_vector_[i].unloading_point)).norm();
                if (dis < 1.0) {
                    car_vector_[i].waybill_list.push_back(new BillStatus(wb));
                    car_vector_[i].ordered_waybill_list_.push_back(new BillStatus(wb));
                    continue;
                }
            }
        }
        new_waybill_set_ = true;
    }
    // for(auto &wb : waybill_list_) {
    //     for (int i = 0; i < car_vector_.size(); i++) {
    //         // cout << positionToVector3d(wb.target_pos).transpose() << endl;
    //         // cout << positionToVector3d(car_vector_[i].unloading_point).transpose() << endl;
    //         // cout << endl;
    //         // wyc 1019: 每次进回调清空再添加
    //         double dis = (positionToVector3d(wb.target_pos) - 
    //                         positionToVector3d(car_vector_[i].unloading_point)).norm();
    //         if (dis < 1.0) {
    //             car_vector_[i].waybill_list.push_back(new BillStatus(wb));
    //             continue;
    //         }
    //     }
    // }


    for (int i = 0; i < car_vector_.size(); i++) {
        car_vector_[i].physical_statu = msg->cars[findCarIndexBySN(msg->cars, car_vector_[i].sn)];
    }
    for (int i = 0; i < drone_vector_.size(); i++) {
        drone_vector_[i].physical_statu = msg->drones[findDroneIndexBySN(msg->drones, drone_vector_[i].sn)];
    }
}

/**
 * @brief 指令结果回调
*/
void cmdResponseCallback(const UserCmdResponse::ConstPtr& msg) {
    cmd_response_type_ = msg->type;
    cout << "#################### cmd response callback start ####################" << endl; 
    cout << "Ros Received cmd response:" << cmd_response_type_ << endl;
    cout << " description: " << msg->description << endl;
    cout << "####################  cmd response callback end  ####################" << endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "race_scar_node");
    ros::NodeHandle nh;

    // ========== ros ==========
    mtuav_timer_ = nh.createTimer(ros::Duration(0.5), mtuavFSM);
    cmd_pub_ = nh.advertise<UserCmdRequest>("/cmd_exec", 10000);
    info_sub_ = nh.subscribe("/panoramic_info", 10, panoramicInfoCallback);
    cmd_resp_sub_ = nh.subscribe("/cmd_resp", 10, cmdResponseCallback);

    ros::Duration(0.1).sleep();

    // ========== init ==========
    if (!initMtuav()) {
        cout << "Init mtuav fail!" << endl; 
        return 1;
    }

    time_start_ = ros::Time::now();

    ros::spin();
    return 0;
}
