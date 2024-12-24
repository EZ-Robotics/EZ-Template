#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"

using namespace ez;

void Drive::check_imu_task()
{
    std::vector<int> deleteIndexes = {};
    for(int i = 0; i < bad_imus.size(); i++)
    {
        auto& [imuPtr, start_time] = bad_imus[i];
        if(pros::millis() - start_time > util::DISCONNECT_THRESHOLD)
        {
            delete imuPtr; //always invalid
            deleteIndexes.push_back(i);
        }
        else if(imuPtr->get_status() != pros::ImuStatus::error && errno != PROS_ERR)//not sure if errno is needed yet. I think it is??
        {
            good_imus.push_front(imuPtr);
            deleteIndexes.push_back(i);
            imu = imuPtr;
        }
    }
    for(auto i : deleteIndexes)
    {
        bad_imus.erase(bad_imus.begin() + i);
    }
    
    if((imu->get_status() == pros::ImuStatus::error || errno == PROS_ERR) && !good_imus.empty())//not sure if errno is needed yet. I think it is??
    {
        //switch due to an error
        bad_imus.push_front(std::make_tuple(good_imus.front(), pros::millis()));
        good_imus.pop_front();
        if (!good_imus.empty())
        {
            imu = good_imus.front();
        }
    }  
}