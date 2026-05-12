#ifndef MAIN_CAR_BASE_H_
#define MAIN_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class MainCar : public VehicleBase // 主车
    {
    public:
        MainCar();
        void vechicle_cartesin_to_frent(const Referline &refer_line) override;
        void vechicle_cartesin_to_frent_2path(const LocalPath &local_path,
                                              const Referline &refer_line,
                                              const std::shared_ptr<VehicleBase> &car) override; // 定位点在参考线上的投影点参数
     };
} // namespace Planning
#endif // MAIN_CAR_BASE_H_