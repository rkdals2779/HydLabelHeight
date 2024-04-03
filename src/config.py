
class __Params__:
    target_region_pixel_ltrb = [680, 900, 1349, 1200]
    section_layout = [3, 1]     # columns, rows
    section_shape = [(target_region_pixel_ltrb[2] - target_region_pixel_ltrb[0])/section_layout[0],
                     (target_region_pixel_ltrb[3] - target_region_pixel_ltrb[1])/section_layout[1],]


DATASET = {'path': '/media/falcon/50fe2d19-4535-4db4-85fb-6970f063a4a11/ActiveDrive/HYD_2023/hyundai_v3_FINAL/train'}

PCD_SPLIT = {
    'camera_param': {'fx': 1508.359, 'fy': 1500.488, 'cx': 1031.306, 'cy': 564.108},
    'target_region_pixel_ltrb': __Params__.target_region_pixel_ltrb,
    'section_layout': __Params__.section_layout,  # target region is divided by layout
    'section_shape': __Params__.section_shape,  # section size in pixels
}

PLANE_ESTIM = {
    'default_plane_normal': [0, 0, 1],
    'default_plane_height': 1.0,
}

HEIGHT_ESTIM = {
    "category_threshold": {"bump": 0.8, "steel": 0.08, "manhole": 0.05, "pothole": 0.05}
}


