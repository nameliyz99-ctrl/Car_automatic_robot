#ifndef OBS_CAR_BASE_H_
#define OBS_CAR_BASE_H_

#include "vehicle_info_base.h"

namespace Planning
{
    class ObsCar : public VehicleBase // 障碍车
    {
    public:
        ObsCar(const int &id); // id号

        void vechicle_cartesin_to_frent(const Referline &refer_line) override;
        void vechicle_cartesin_to_frent_2path(const LocalPath &local_path,
                                              const Referline &refer_line,
                                              const std::shared_ptr<VehicleBase> &car) override; // 定位点在参考线上的投影点参数
    };
} // namespace Planning
#endif // OBS_CAR_BASE_H_