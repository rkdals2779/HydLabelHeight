import numpy as np


class PcdSplitter:
    def __init__(self, cfg):
        self.camera_params = cfg["camera_param"]
        self.target_region = cfg["target_region_pixel_ltrb"]
        self.section_layout = cfg['section_layout']

    def split_pcd_into_sections(self, pcd):
        pcd = self.compute_pixel(pcd)
        pcd_splits, section_xy_border = self.split_sections(pcd)
        return pcd_splits, section_xy_border

    def compute_pixel(self, pcd):
        """
        :param pcd: (N, 3) [x, y, z]
        :return: (N, 5) [x, y, z, u, v]
        """
        intrinsic = np.array([[self.camera_params["fx"], 0, self.camera_params["cx"]],
                              [0, self.camera_params["fy"], self.camera_params["cy"]],
                              [0, 0, 1]])
        transformed_points = (intrinsic @ pcd.T).T
        pixels = np.array(transformed_points[:, :2] / transformed_points[:, 2:3], dtype=np.int32)
        pixels_tl_corner_mask = np.logical_and(pixels[:, 1] > self.target_region[1],
                                               pixels[:, 0] > self.target_region[0])
        pixels_br_corner_mask = np.logical_and(pixels[:, 1] < self.target_region[3],
                                               pixels[:, 0] < self.target_region[2])
        pixels_inside_target = np.logical_and(pixels_tl_corner_mask, pixels_br_corner_mask)
        pcd = pcd[pixels_inside_target]
        pixels = pixels[pixels_inside_target]
        pcd_pixels = np.concatenate([pcd, pixels], axis=-1)
        return pcd_pixels

    def split_sections(self, pcd):
        """
        :param pcd: (N, 5) [x, y, z, u, v]
        :return: [[(N0,5), (N1,5), ...],   : first rows of sections
                  [(Ni,5), (Ni+1,5), ...], : second rows of sections
                 ]
        """
        x_start, y_start, x_end, y_end = self.target_region
        x_border = np.linspace(x_start, x_end, self.section_layout[0] + 1, dtype=np.int16)
        y_border = np.linspace(y_start, y_end, self.section_layout[1] + 1, dtype=np.int16)
        pcd_splits = []
        for y1, y2 in zip(y_border[:-1], y_border[1:]):
            row_splits = []
            for x1, x2 in zip(x_border[:-1], x_border[1:]):
                section_ltrb = x1, y1, x2, y2
                section_pcd = self.extract_section(pcd, section_ltrb)
                row_splits.append(section_pcd)
            pcd_splits.append(row_splits)

        section_xy_border = (x_border, y_border)
        return pcd_splits, section_xy_border

    def extract_section(self, pcd, section_ltrb):
        mask = np.logical_and(np.logical_and((section_ltrb[0] < pcd[:, 3]), (pcd[:, 3] < section_ltrb[2])),
                              np.logical_and((section_ltrb[1] < pcd[:, 4]), (pcd[:, 4] < section_ltrb[3]))
                              )
        return pcd[mask]
