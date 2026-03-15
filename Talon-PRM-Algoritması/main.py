from TeknofestPRMPlanner import TeknofesPRMPlanner, show_path_simple

demo_data = {
    "hss_koordinat_bilgileri": [
        {"id": 0, "hssEnlem": 40.23400000, "hssBoylam": 29.00744677, "hssYaricap": 50},
        {"id": 1, "hssEnlem": 40.23351019, "hssBoylam": 28.99976492, "hssYaricap": 50},
        {"id": 2, "hssEnlem": 40.23105297, "hssBoylam": 29.00744677, "hssYaricap": 75},
        {"id": 3, "hssEnlem": 40.23090554, "hssBoylam": 28.99976492, "hssYaricap": 120},
        {"id": 4, "hssEnlem": 40.23180000, "hssBoylam": 29.00300000, "hssYaricap": 40},
        {"id": 5, "hssEnlem": 40.23351019, "hssBoylam": 29.00300000, "hssYaricap": 60}
    ],
    "mission_info": {
        "start_gps": {"lat": 40.23000000, "lon": 28.99500000},
        "goal_gps": {"lat": 40.23500000, "lon": 29.01000000},
        "max_altitude": 120,
        "min_altitude": 30,
        "safety_margin": 15
    }
}

start_gps = demo_data["mission_info"]["start_gps"]
goal_gps = demo_data["mission_info"]["goal_gps"]

# Yol planla
planner = TeknofesPRMPlanner()
result = planner.plan_path(demo_data, start_gps, goal_gps)

show_path_simple(result, planner, create_waypoints=True, waypoint_filename="teknofest_path.waypoints")


