import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

max_data_mcs11 = 8.44
max_data_mcs7 = 5.0625

tcr_Bps = 29
tcm_Bps = 42
spat_Bps = 3400 # Safety Critical
map_Bps = 700
bsm_Bps = 3400 # Safety Critical
psm_Bps = 2700 # Safety Critical
mom_Bps = 2700
mpat_Bps = 5700
mreq_Bps = 3200
mresp_Bps = 1500

tcr_Bps_max = 78
tcm_Bps_max = 110
spat_Bps_max = 14000 # Safety Critical
map_Bps_max = 1400
bsm_Bps_max = 14000 # Safety Critical
psm_Bps_max = 14000 # Safety Critical
mom_Bps_max = 10500
mpat_Bps_max = 8500
mreq_Bps_max = 11520
mresp_Bps_max = 10020


# Safety Critical
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
    Bps_per_leader_safety = bsm_Bps
    Bps_per_follower_safety = bsm_Bps
    Bps_per_veh = (Bps_per_follower * (platoon_size - 1) + Bps_per_leader) / platoon_size
    Bps_per_veh_safety = (Bps_per_follower_safety * (platoon_size - 1) + Bps_per_leader_safety) / platoon_size
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    safety_Bps = num_vehicles * Bps_per_veh_safety
    return total_Bps, safety_Bps, num_vehicles


def simulation_workzone(speed, headway, num_lanes, radio_range,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    veh_per_m = vehicles_per_m(speed, headway)
    Bps_per_veh = bsm_Bps + mpat_Bps + tcr_Bps + tcm_Bps
    Bps_per_veh_safety = bsm_Bps
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    safety_Bps = num_vehicles * Bps_per_veh_safety
    return total_Bps, safety_Bps, num_vehicles


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
    Bps_per_veh_safety = bsm_Bps
    Bps_per_rsu_safety = spat_Bps
    
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
    safety_Bps = Bps_per_veh_safety * num_vehicles + Bps_per_rsu_safety * num_intersections
    safety_Bps += add_Bps_veh + add_Bps_rsu
    return total_Bps, safety_Bps, num_vehicles
    
    
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
    
    Bps_per_veh = sdsm_Bps(visible_vehicles_per_vehicle) + bsm_Bps + mpat_Bps + tcr_Bps
    Bps_per_veh_safety = sdsm_Bps(visible_vehicles_per_vehicle) + bsm_Bps
    total_Bps = Bps_per_veh * num_vehicles
    safety_Bps = Bps_per_veh_safety * num_vehicles
    return total_Bps, safety_Bps, num_vehicles


def simulation_combined(speed, headway, num_lanes, radio_range, intersection_spacing, stopped_fill_prop=0.5,
                             road_width=12, vehicle_len=8, num_ped=15,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps):
    total_Bps, safety_Bps, num_vehicles = simulation_intersections(speed, headway, num_lanes, radio_range, intersection_spacing,
                                   stopped_fill_prop=stopped_fill_prop, road_width=road_width,
                                   vehicle_len=vehicle_len, cp=True, num_ped=num_ped,
                          tcr_Bps=tcr_Bps, tcm_Bps=tcm_Bps, spat_Bps=spat_Bps, map_Bps=map_Bps, bsm_Bps=bsm_Bps,
                          psm_Bps=psm_Bps, mom_Bps=mom_Bps, mpat_Bps=mpat_Bps, mreq_Bps=mreq_Bps, mresp_Bps=mresp_Bps)
    return total_Bps, safety_Bps, num_vehicles


def plot_scenario(x, y, y_max):
    plt.plot(x, y, color='k', linestyle='solid', label='Typical Throughput')
    plt.plot(x, y_max, color='k', linestyle='dashed', label='Maximum Throughput')
    plt.plot(x, [max_data_mcs11]*len(y), color='grey', linestyle='dashdot', label='MCS11 Maximum')
    plt.plot(x, [max_data_mcs7]*len(y), color='grey', linestyle='dotted', label='MCS7 Maximum')


def plot_scenario_with_veh_count(speed, sim_Bps, sim_max_Bps, title, x1_label, x2_label, ylabel, locs, save_name=None):
    MBps = sim_Bps[1:-1, 0] / 1000000
    safety_MBps = sim_Bps[1:-1, 1] / 1000000
    max_MBps = sim_max_Bps[1:-1, 0] / 1000000
    max_safety_MBps = sim_max_Bps[1:-1, 1] / 1000000
    num_vehicles = sim_Bps[:, 2]

    fig, ax1 = plt.subplots(constrained_layout=True)
    ax1.set_title(title)
    ax1.set_ylabel(ylabel)
    ax1.set_xlabel(x1_label)
    ax1.plot(speed[1:-1], max_MBps, color='r', marker='o', linestyle='dashed', label='Maximum Throughput')
    ax1.plot(speed[1:-1], max_safety_MBps, color='r', marker='o', linestyle='solid', label='Maximum Safety Throughput')
    ax1.plot(speed[1:-1], MBps, color='k', marker='s', linestyle='dashed', label='Typical Throughput')
    ax1.plot(speed[1:-1], safety_MBps, color='k', marker='s', linestyle='solid', label='Typical Safety Throughput')
    ax1.plot(speed[1:-1], [max_data_mcs11]*len(MBps), color='grey', marker='.', linestyle='dashdot', label='MCS11 Maximum')
    ax1.plot(speed[1:-1], [max_data_mcs7]*len(MBps), color='grey', marker='.', linestyle='dotted', label='MCS7 Maximum')

    def forward(x):
        return np.interp(x, speed, num_vehicles)

    def inverse(x):
        return np.interp(x, np.flip(num_vehicles), np.flip(speed))

    ax2 = ax1.secondary_xaxis('top', functions=(forward, inverse))
    # ax2.set_xscale("log")
    # ax2.xaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax2.xaxis.set_minor_locator(ticker.FixedLocator(locs))
    ax2.xaxis.set_major_locator(ticker.NullLocator())
    ax2.xaxis.set_minor_formatter(ticker.ScalarFormatter())
    ax2.set_xlabel(x2_label)
    plt.legend()
    if save_name is not None:
        fig.savefig(save_name)
    plt.show()


def main():
    mph_to_mps = 0.44704

    headway = 1.2
    headway_intra_platoon = 0.6
    num_lanes = 12
    radio_range = 1600

    city_speed = 30
    city_num_lanes = num_lanes // 2
    city_radio_range = 400
    # Actual used intersection spacing range is 150 - 1200, but we need some data to extrapolate the axes from
    intersection_spacing_range = [100, 150, 200, 300, 500, 800, 1200, 1500]

    messages_normal = {'tcr_Bps': tcr_Bps, 'tcm_Bps': tcm_Bps, 'spat_Bps': spat_Bps, 'map_Bps': map_Bps, 'bsm_Bps': bsm_Bps, 'psm_Bps': psm_Bps, 'mom_Bps': mom_Bps, 'mpat_Bps': mpat_Bps, 'mreq_Bps': mreq_Bps, 'mresp_Bps': mresp_Bps}
    messages_max = {'tcr_Bps': tcr_Bps_max, 'tcm_Bps': tcm_Bps_max, 'spat_Bps': spat_Bps_max, 'map_Bps': map_Bps_max, 'bsm_Bps': bsm_Bps_max, 'psm_Bps': psm_Bps_max, 'mom_Bps': mom_Bps_max, 'mpat_Bps': mpat_Bps_max, 'mreq_Bps': mreq_Bps_max, 'mresp_Bps': mresp_Bps_max}

    # Actual used speed range is 10 - 60, but we need some data to extrapolate the axes from
    speed_range = [5, 10, 20, 30, 40, 50, 60, 75]
    sim1_data = np.zeros((len(speed_range), 3)) # Bps, safety_Bps, num_vehicles
    sim1_max_data = np.zeros((len(speed_range), 3)) # max_Bps, max_safety_Bps, max_num_vehicles
    sim2_data = np.zeros((len(speed_range), 3)) # Bps, safety_Bps, num_vehicles
    sim2_max_data = np.zeros((len(speed_range), 3)) # max_Bps, max_safety_Bps, max_num_vehicles
    sim4_data = np.zeros((len(speed_range), 3)) # Bps, safety_Bps, num_vehicles
    sim4_max_data = np.zeros((len(speed_range), 3)) # max_Bps, max_safety_Bps, max_num_vehicles

    sim3_data = np.zeros((len(intersection_spacing_range), 3)) # Bps, safety_Bps, num_vehicles
    sim3_max_data = np.zeros((len(intersection_spacing_range), 3)) # max_Bps, max_safety_Bps, max_num_vehicles
    sim5_data = np.zeros((len(intersection_spacing_range), 3)) # Bps, safety_Bps, num_vehicles
    sim5_max_data = np.zeros((len(intersection_spacing_range), 3)) # max_Bps, max_safety_Bps, max_num_vehicles

    for i, speed in enumerate(speed_range):
        sim1_data[i] = simulation_platooning(speed * mph_to_mps, headway, headway_intra_platoon, num_lanes, radio_range, **messages_normal)
        sim1_max_data[i] = simulation_platooning(speed * mph_to_mps, headway, headway_intra_platoon, num_lanes, radio_range, **messages_max)

        sim2_data[i] = simulation_workzone(speed * mph_to_mps, headway, num_lanes - 2, radio_range, **messages_normal)
        sim2_max_data[i] = simulation_workzone(speed * mph_to_mps, headway, num_lanes - 2, radio_range, **messages_max)

        sim4_data[i] = simulation_cp(speed * mph_to_mps, headway, num_lanes, radio_range, **messages_normal)
        sim4_max_data[i] = simulation_cp(speed * mph_to_mps, headway, num_lanes, radio_range, **messages_max)

    for i, intersection_spacing in enumerate(intersection_spacing_range):
        sim_args_city = [city_speed * mph_to_mps, headway, city_num_lanes, city_radio_range, intersection_spacing]
        sim3_data[i] = simulation_intersections(*sim_args_city, **messages_normal)
        sim3_max_data[i] = simulation_intersections(*sim_args_city, **messages_max)

        sim5_data[i] = simulation_combined(*sim_args_city, **messages_normal)
        sim5_max_data[i] = simulation_combined(*sim_args_city, **messages_max)

    # locs1 = np.array([750, 1000, 1250, 1500])
    # locs2 = np.array([250, 500, 750, 1000, 1250])
    # locs4 = np.array([500, 600, 700, 800, 900, 1000, 1250])
    # locs3 = np.array([100, 200, 300, 400, 500, 600, 700])
    # locs5 = np.array([100, 200, 300, 400, 500, 600, 700])
    # plot_scenario_with_veh_count(speed_range, sim1_data, sim1_max_data, "Platooning Message Throughput (range 800 m)",
    #                              "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs1, save_name='spectrum_platooning_short.png')
    # plot_scenario_with_veh_count(speed_range, sim2_data, sim2_max_data, "Workzone Message Throughput (range 800 m)",
    #                              "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs2, save_name='spectrum_wz_short.png')
    # plot_scenario_with_veh_count(speed_range, sim4_data, sim4_max_data, "Cooperative Perception Message Throughput (range 800 m)",
    #                              "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs4, save_name='spectrum_cp_short.png')
    # plot_scenario_with_veh_count(speed_range, sim3_data, sim3_max_data, "City Intersections Message Throughput (range 230 m)",
    #                              "Intersection spacing (m)", "Number of vehicles", "Throughput (MB/s)", locs3, save_name='spectrum_int_short.png')
    # plot_scenario_with_veh_count(speed_range, sim5_data, sim5_max_data, "City Intersections CP Message Throughput (range 230 m)",
    #                              "Intersection spacing (m)", "Number of vehicles", "Throughput (MB/s)", locs5, save_name='spectrum_int_cp_short.png')

    locs1 = np.array([1500, 2000, 2500, 3000])
    locs2 = np.array([800, 1000, 1250, 1500, 2000])
    locs4 = np.array([1000, 1250, 1500, 2000, 3000])
    locs3 = np.array([300, 400, 500, 750, 1000, 2000])
    locs5 = np.array([300, 400, 500, 750, 1000, 2000])
    plot_scenario_with_veh_count(speed_range, sim1_data, sim1_max_data, "Platooning Message Throughput (range 1600 m)",
                                 "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs1, save_name='spectrum_platooning_long.png')
    plot_scenario_with_veh_count(speed_range, sim2_data, sim2_max_data, "Workzone Message Throughput (range 1600 m)",
                                 "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs2, save_name='spectrum_wz_long.png')
    plot_scenario_with_veh_count(speed_range, sim4_data, sim4_max_data, "Cooperative Perception Message Throughput (range 1600 m)",
                                 "Vehicle Speed (mph)", "Number of vehicles", "Throughput (MB/s)", locs4, save_name='spectrum_cp_long.png')
    plot_scenario_with_veh_count(intersection_spacing_range, sim3_data, sim3_max_data, "City Intersections Message Throughput (range 400 m)",
                                 "Intersection spacing (m)", "Number of vehicles", "Throughput (MB/s)", locs3, save_name='spectrum_int_long.png')
    plot_scenario_with_veh_count(intersection_spacing_range, sim5_data, sim5_max_data, "City Intersections CP Message Throughput (range 400 m)",
                                 "Intersection spacing (m)", "Number of vehicles", "Throughput (MB/s)", locs5, save_name='spectrum_int_cp_long.png')

    print('done')


if __name__ == '__main__':
    main()
