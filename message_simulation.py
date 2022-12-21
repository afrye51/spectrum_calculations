import numpy as np
import matplotlib.pyplot as plt


class message_simulation():
    def __init__(self, num_intersections, num_cars):
        self.num_intersections = num_intersections
        self.num_cars = num_cars
        self.bw = 0
        self.tcr_b = 10
        self.tcr_hz = 10
        self.map_b = 10
        self.map_hz = 10

    def simulate(self):
        print('creating results')

    def add_intersection(self):
        self.bw += self.map_b * self.map_hz

    def plot_results(self):
        xpoints = np.array([0, 6])
        ypoints = np.array([0, 250])

        plt.title("message bandwidth vs vehicle headway")
        plt.xlabel("Bandwidth (MB/s)")
        plt.ylabel("Vehicle Headway (seconds)")
        plt.plot(xpoints, ypoints)
        plt.show()

max_data_mBps = 67.5

tcr_Bps = 4
tcm_Bps = 17
spat_Bps = 2700
map_Bps = 450
bsm_Bps = 2700
psm_Bps = 2000
mom_Bps = 2000
mpat_Bps = 5000
mreq_Bps = 2500
mresp_Bps = 800


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


def simulation_platooning(speed, inter_platoon_headway, intra_platoon_headway, num_lanes, radio_range, platoon_size=10):
    veh_per_m = vehicles_per_m_platooning(speed, inter_platoon_headway, intra_platoon_headway)
    Bps_per_leader = bsm_Bps + mpat_Bps + mom_Bps + tcr_Bps
    Bps_per_follower = bsm_Bps + mpat_Bps + mom_Bps + tcr_Bps
    Bps_per_veh = (Bps_per_follower * (platoon_size - 1) + Bps_per_leader) / platoon_size
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    return total_Bps


def simulation_workzone(speed, headway, num_lanes, radio_range):
    veh_per_m = vehicles_per_m(speed, headway)
    Bps_per_veh = bsm_Bps + mpat_Bps + tcr_Bps + tcm_Bps
    num_vehicles = veh_per_m * num_lanes * radio_range * 2
    total_Bps = num_vehicles * Bps_per_veh
    return total_Bps


def simulation_intersections(speed, headway, num_lanes, radio_range, intersection_spacing, stopped_fill_prop=0.5, 
                             road_width=12, vehicle_len=8, cp=False, num_ped=0):
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
    
    
def simulation_cp(speed, headway, num_lanes, radio_range, vehicle_len=8):
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
                             road_width=12, vehicle_len=8, num_ped=15):
    total_Bps = simulation_intersections(speed, headway, num_lanes, radio_range, intersection_spacing,
                                   stopped_fill_prop=stopped_fill_prop, road_width=road_width,
                                   vehicle_len=vehicle_len, cp=True, num_ped=num_ped)
    return total_Bps
    

# Assuming stopped vehicles in a traffic jam, compute the number of vehicles within a certain radius
#   num_lanes: total number of lanes on the road
#   radius: radius in km to count cars in (radius of cv messages)
#   spacing: time headway between vehicles in meters
#   offset: distance in km from the road to the center of the circle (making the road a chord)
#   vehicle_len: length in meters of each vehicle
def compute_num_cars_traffic_jam(num_lanes, radius, spacing, offset=0, vehicle_len=8):
    clearance = spacing + vehicle_len
    density_per_km = 1000 / clearance
    road_length_in_radius = 2 * np.sqrt(radius**2 + offset**2)
    return num_lanes * (density_per_km * road_length_in_radius)


# Assuming free traffic flow, compute the number of vehicles within a certain radius
#   num_lanes: total number of lanes on the road
#   radius: radius in km to count cars in (radius of cv messages)
#   speed: speed of vehicles in m/s
#   headway: time headway between vehicles in seconds
#   offset: distance in km from the road to the center of the circle (making the road a chord)
#   vehicle_len: length in meters of each vehicle
def compute_num_cars_free_flow(num_lanes, radius, speed, headway, offset=0, vehicle_len=8):
    clearance = headway * speed + vehicle_len
    density_per_km = 1000 / clearance
    road_length_in_radius = 2 * np.sqrt(radius**2 + offset**2)
    return num_lanes * (density_per_km * road_length_in_radius)


def plot_speed_data(speeds, throughputs, title, ylabel='Throughput (MB/s)', xlabel='Vehicle Speed (mph)'):
    print('hi')


def main():
    mph_to_mps = 0.44704

    headway = 1.2
    headway_intra_platoon = 0.6
    num_lanes = 12
    radio_range = 2700

    city_speed = 30
    city_num_lanes = num_lanes // 2
    city_radio_range_fixed = 600
    city_radio_range_range = [800, 600, 400, 200]
    intersection_spacing_range = [150, 200, 300, 500, 800, 1200]

    speed_range = [10, 20, 30, 40, 50, 60]
    sim1_MBps = []
    sim2_MBps = []
    sim3_MBps = []
    sim4_MBps = []
    sim5_MBps = []

    for speed in speed_range:
        sim1_MBps.append(simulation_platooning(speed * mph_to_mps, headway, headway_intra_platoon, num_lanes, radio_range) / 1000000)
        sim2_MBps.append(simulation_workzone(speed * mph_to_mps, headway, num_lanes - 2, radio_range) / 1000000)
        sim4_MBps.append(simulation_cp(speed * mph_to_mps, headway, num_lanes, radio_range) / 1000000)

    for intersection_spacing in intersection_spacing_range:
        sim_args_city = [city_speed * mph_to_mps, headway, city_num_lanes, city_radio_range_fixed, intersection_spacing]
        sim3_MBps.append(simulation_combined(*sim_args_city) / 1000000)

    for city_radio_range in city_radio_range_range:
        sim5_tmp_MBps = []
        for intersection_spacing in intersection_spacing_range:
            sim_args_city = [city_speed * mph_to_mps, headway, city_num_lanes, city_radio_range, intersection_spacing]
            sim5_tmp_MBps.append(simulation_combined(*sim_args_city) / 1000000)
        sim5_MBps.append(sim5_tmp_MBps)

    plt.title("Platooning Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plt.plot(speed_range, sim1_MBps)
    plt.show()

    plt.title("Workzone Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plt.plot(speed_range, sim2_MBps)
    plt.show()

    plt.title("Cooperative Perception Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Vehicle Speed (mph)")
    plt.plot(speed_range, sim4_MBps)
    plt.show()

    plt.title("City Intersections Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Intersection spacing (m)")
    plt.plot(intersection_spacing_range, sim3_MBps)
    plt.show()

    plt.title("City Intersections Cooperative Perception Message Throughput")
    plt.ylabel("Throughput (MB/s)")
    plt.xlabel("Intersection spacing (m)")
    colors = ['y', 'orange', 'r', 'k']
    for i, city_radio_range in enumerate(city_radio_range_range):
        plt.plot(intersection_spacing_range, sim5_MBps[i], color=colors[i], label=f'range: {city_radio_range} m')
    plt.legend()
    plt.show()
    # plt.savefig('foo.png')

    print('done')


if __name__ == '__main__':
    main()
