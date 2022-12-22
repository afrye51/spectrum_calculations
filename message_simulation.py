import numpy as np
import matplotlib.pyplot as plt

max_data_mcs11 = 67.5
max_data_mcs7 = 40.5

tcr_Bps = 29
tcm_Bps = 42
spat_Bps = 3400
map_Bps = 700
bsm_Bps = 3400
psm_Bps = 2700
mom_Bps = 2700
mpat_Bps = 5700
mreq_Bps = 3200
mresp_Bps = 1500

tcr_Bps_max = 78
tcm_Bps_max = 110
spat_Bps_max = 14000
map_Bps_max = 1400
bsm_Bps_max = 14000
psm_Bps_max = 14000
mom_Bps_max = 10500
mpat_Bps_max = 8500
mreq_Bps_max = 11520
mresp_Bps_max = 10020


def sdsm_Bps(num_objects):
    return 500 + 300 * num_objects


# inter 11, intra 1
# 10-car platoon -> 11 + 9 = 20, 10/20 = 0.5 cars/m
# 10 1-cars -> 10 * 11 = 110, 10/110 = 0.09 cars/m
# 10-car platoon + 2 cars -> 20 + 22 = 42, 12/42 = 0.25 cars/m
def vehicles_per_m_platooning(speed, inter_platoon_headway, intra_platoon_headway, platoon_size=8, vehicle_len=8):
    inter_platoon_clearance = inter_platoon_headway * speed + vehicle_len
    intra_platoon_clearance = intra_platoon_headway * speed + vehicle_len
    platoon_clearance = inter_platoon_clearance + (platoon_size - 1) * intra_platoon_clearance
    vehicles_per_m = platoon_size / platoon_clearance
    return vehicles_per_m


def vehicles_per_m(speed, headway, vehicle_len=8):
    return vehicles_per_m_platooning(speed, headway, headway, platoon_size=1, vehicle_len=vehicle_len)


def simulation_platooning(speed, inter_platoon_headway, intra_platoon_headway, num_lanes, radio_range, platoon_size=10,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    veh_per_m = vehicles_per_m_platooning(speed, inter_platoon_headway, intra_platoon_headway)
    Bps_per_leader = bsm_Bps + mpat_Bps + mom_Bps + tcr_Bps
    Bps_per_follower = bsm_Bps + mpat_Bps + mom_Bps + tcr_Bps
    Bps_per_veh = (Bps_per_follower * (platoon_size - 1) + Bps_per_leader) / platoon_size
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    return total_Bps


def simulation_workzone(speed, headway, num_lanes, radio_range,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    veh_per_m = vehicles_per_m(speed, headway)
    Bps_per_veh = bsm_Bps + mpat_Bps + tcr_Bps + tcm_Bps
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    return total_Bps


def simulation_intersections(speed, headway, num_lanes, radio_range, intersection_spacing, stopped_fill_prop=0.5, 
                             road_width=12, vehicle_len=8, cp=False, num_ped=0,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    vehicles_per_m_driving = vehicles_per_m(speed, headway)
    vehicles_per_m_stopped = ((intersection_spacing - road_width) * stopped_fill_prop / vehicle_len) / intersection_spacing
    
    # Approximately 1 intersection per unit area. If you fill the circle with a grid, 
    #   the number of squares will be the circle area / spacing, and each grid owns one of its corners.
    # This approximation should be good for >10 intersections
    num_intersections = np.pi * radio_range**2 / intersection_spacing**2
    road_length_driving = num_intersections * intersection_spacing
    road_length_stopped = num_intersections * intersection_spacing
    num_vehicles = (vehicles_per_m_driving * road_length_driving + vehicles_per_m_stopped * road_length_stopped) * num_lanes

    Bps_per_veh = bsm_Bps + mpat_Bps + tcr_Bps
    Bps_per_rsu = spat_Bps + map_Bps + mom_Bps
    
    add_Bps_veh = 0
    add_Bps_rsu = 0
    if cp:
        num_lanes_center = max(0, num_lanes - 2)
        num_lanes_side = min(2, num_lanes)
        num_detections_center = num_lanes * 2 + min(2, max(0, num_lanes - 1))
        num_detections_side = num_lanes * 2 + min(1, max(0, num_lanes - 1))
        num_detections_front = num_lanes + min(2, max(0, num_lanes - 1))
        # cars center driving
            # 8 bsms
        num_cars_driving_center = (vehicles_per_m_driving * num_lanes_center) * intersection_spacing * num_intersections
        add_Bps_veh += num_cars_driving_center * sdsm_Bps(num_detections_center)
        # cars side driving
            # 10 bsms
            # num_ped psms
        num_cars_driving_side = (vehicles_per_m_driving * num_lanes_side) * intersection_spacing * num_intersections
        add_Bps_veh += num_cars_driving_side * sdsm_Bps(num_detections_side + num_ped)
        # cars center stopped
            # 8 bsms
        num_cars_stopped_center = (vehicles_per_m_stopped * num_lanes_center) * intersection_spacing * num_intersections
        add_Bps_veh += num_cars_stopped_center * sdsm_Bps(num_detections_center)
        # cars side stopped
            # 7 bsms
        num_cars_stopped_side = (vehicles_per_m_driving * num_lanes_side) * intersection_spacing * num_intersections
        add_Bps_veh += num_cars_stopped_side * sdsm_Bps(num_detections_side)
        # cars front stopped (3 per intersection)
            # side vehicles per intersection bsms
        num_cars_stopped_front = num_lanes * num_intersections
        add_Bps_veh += num_cars_stopped_front * \
                       sdsm_Bps(vehicles_per_m_driving * intersection_spacing + num_detections_front + num_ped)
        
        # intersections detecting pedestrians
        add_Bps_rsu += num_intersections * sdsm_Bps(num_ped * 2)
    
    total_Bps = Bps_per_veh * num_vehicles + Bps_per_rsu * num_intersections
    total_Bps += add_Bps_veh + add_Bps_rsu
    return total_Bps
    
    
def simulation_cp(speed, headway, num_lanes, radio_range, vehicle_len=8,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    # convert it to one single direction 2x long to make the math easier
    road_length = 4 * radio_range
    num_lanes = num_lanes / 2
    
    # interior lanes are 8 detection per vehicle, exterior lanes are 7 detections per vehicle
    num_lanes_center = max(0, num_lanes - 2)
    num_lanes_side = min(2, num_lanes)
    num_detections_center = num_lanes * 2 + min(2, max(0, num_lanes - 1))
    num_detections_side = num_lanes * 2 + min(1, max(0, num_lanes - 1))
    visible_vehicles_per_vehicle = (num_detections_center*num_lanes_center + num_detections_side*num_lanes_side) / num_lanes

    veh_per_m = vehicles_per_m(speed, headway, vehicle_len=vehicle_len)
    num_vehicles = veh_per_m * num_lanes * road_length
    
    msgs_per_veh = sdsm_Bps(visible_vehicles_per_vehicle) + bsm_Bps + mpat_Bps + tcr_Bps
    num_msgs = msgs_per_veh * num_vehicles
    return num_msgs


def simulation_combined(speed, headway, num_lanes, radio_range, intersection_spacing, stopped_fill_prop=0.5,
                             road_width=12, vehicle_len=8, num_ped=15,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    total_Bps = simulation_intersections(speed, headway, num_lanes, radio_range, intersection_spacing,
                                   stopped_fill_prop=stopped_fill_prop, road_width=road_width,
                                   vehicle_len=vehicle_len, cp=True, num_ped=num_ped,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps)
    return total_Bps


def plot_scenario(x, y, y_max):
    plt.plot(x, y, color='k', linestyle='solid', label='Typical Throughput')
    plt.plot(x, y_max, color='k', linestyle='dashed', label='Maximum Throughput')
    plt.plot(x, [max_data_mcs11]*len(y), color='grey', linestyle='dashdot', label='MCS11 Maximum')
    plt.plot(x, [max_data_mcs7]*len(y), color='grey', linestyle='dotted', label='MCS7 Maximum')


def main():
    mph_to_mps = 0.44704

    headway = 1.2
    headway_intra_platoon = 0.6
    num_lanes = 12
    radio_range = 2700

    city_speed = 30
    city_num_lanes = num_lanes // 2
    city_radio_range = 600
    intersection_spacing_range = [150, 200, 300, 500, 800, 1200]

    messages_normal = {'tcr_Bps': tcr_Bps, 'tcm_Bps': tcm_Bps, 'spat_Bps': spat_Bps, 'map_Bps': map_Bps, 'bsm_Bps': bsm_Bps, 'psm_Bps': psm_Bps, 'mom_Bps': mom_Bps, 'mpat_Bps': mpat_Bps, 'mreq_Bps': mreq_Bps, 'mresp_Bps': mresp_Bps}
    messages_max = {'tcr_Bps': tcr_Bps_max, 'tcm_Bps': tcm_Bps_max, 'spat_Bps': spat_Bps_max, 'map_Bps': map_Bps_max, 'bsm_Bps': bsm_Bps_max, 'psm_Bps': psm_Bps_max, 'mom_Bps': mom_Bps_max, 'mpat_Bps': mpat_Bps_max, 'mreq_Bps': mreq_Bps_max, 'mresp_Bps': mresp_Bps_max}

    speed_range = [10, 20, 30, 40, 50, 60]
    sim1_MBps = []
    sim1_MBps_max = []
    sim2_MBps = []
    sim2_MBps_max = []
    sim3_MBps = []
    sim3_MBps_max = []
    sim4_MBps = []
    sim4_MBps_max = []
    sim5_MBps = []
    sim5_MBps_max = []

    for speed in speed_range:
        sim1_MBps.append(simulation_platooning(speed * mph_to_mps, headway, headway_intra_platoon, num_lanes, radio_range, **messages_normal) / 1000000)
        sim1_MBps_max.append(simulation_platooning(speed * mph_to_mps, headway, headway_intra_platoon, num_lanes, radio_range, **messages_max) / 1000000)
        sim2_MBps.append(simulation_workzone(speed * mph_to_mps, headway, num_lanes - 2, radio_range, **messages_normal) / 1000000)
        sim2_MBps_max.append(simulation_workzone(speed * mph_to_mps, headway, num_lanes - 2, radio_range, **messages_max) / 1000000)
        sim4_MBps.append(simulation_cp(speed * mph_to_mps, headway, num_lanes, radio_range, **messages_normal) / 1000000)
        sim4_MBps_max.append(simulation_cp(speed * mph_to_mps, headway, num_lanes, radio_range, **messages_max) / 1000000)

    for intersection_spacing in intersection_spacing_range:
        sim_args_city = [city_speed * mph_to_mps, headway, city_num_lanes, city_radio_range, intersection_spacing]
        sim3_MBps.append(simulation_intersections(*sim_args_city, **messages_normal) / 1000000)
        sim3_MBps_max.append(simulation_intersections(*sim_args_city, **messages_max) / 1000000)
        sim5_MBps.append(simulation_combined(*sim_args_city, **messages_normal) / 1000000)
        sim5_MBps_max.append(simulation_combined(*sim_args_city, **messages_max) / 1000000)

    plt.title("Platooning Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plot_scenario(speed_range, sim1_MBps, sim1_MBps_max)
    plt.legend()
    plt.savefig('spectrum_platooning.png')
    plt.show()

    plt.title("Workzone Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plot_scenario(speed_range, sim2_MBps, sim2_MBps_max)
    plt.legend()
    plt.savefig('spectrum_wz.png')
    plt.show()

    plt.title("Cooperative Perception Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plot_scenario(speed_range, sim4_MBps, sim4_MBps_max)
    plt.legend()
    plt.savefig('spectrum_cp.png')
    plt.show()

    plt.title("City Intersections Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Intersection spacing (m)")
    plot_scenario(intersection_spacing_range, sim3_MBps, sim3_MBps_max)
    plt.legend()
    plt.savefig('spectrum_int.png')
    plt.show()

    plt.title("City Intersections Cooperative Perception Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Intersection spacing (m)")
    plot_scenario(intersection_spacing_range, sim5_MBps, sim5_MBps_max)
    plt.legend()
    plt.savefig('spectrum_int_cp.png')
    plt.show()

    # plt.title("City Intersections Cooperative Perception Message Throughput")
    # plt.ylabel("Throughput (MB/s)")
    # plt.xlabel("Intersection spacing (m)")
    # colors = ['y', 'orange', 'r', 'k']
    # for i, city_radio_range in enumerate(city_radio_range_range):
    #     plt.plot(intersection_spacing_range, sim5_MBps[i], color=colors[i], label=f'range: {city_radio_range} m')
    # plt.legend()
    # plt.show()
    # plt.savefig('foo.png')

    print('done')


if __name__ == '__main__':
    main()
