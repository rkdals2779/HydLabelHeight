import numpy as np
import cv2


class HeightVisualizer:
    def __init__(self):
        pass

    def visualize(self, image, pcd, label, section_xy_border):
        image = self.visualize_section(image, section_xy_border)
        image = self.visualize_points(image, pcd)
        if label is not None:
            image = self.visualize_label(image, label)
        cv2.imshow("image", image)
        cv2.waitKey(0)

    def visualize_section(self, image, section_xy_border):
        x_border, y_border = section_xy_border
        for yidx, y1 in enumerate(y_border[:-1]):
            for xidx, x1 in enumerate(x_border[:-1]):
                y2, x2 = y_border[yidx+1], x_border[xidx+1]
                image = cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        return image

    def visualize_points(self, image, pcd):
        pixels = pcd[:, 3:5].astype(np.int16)
        heights = pcd[:, -1]
        colors = self.height_to_color(heights)
        image = self.mark_points(image, pixels, colors)
        return image

    def height_to_color(self, heights):
        color_map = np.zeros((heights.shape[0], 3))
        height_color_pairs = [(-0.05, (255, 0, 0)), (0.05, (0, 255, 0)), (10., (0, 0, 255))]
        low_bound = -10
        for up_bound, color in height_color_pairs:
            color_map[np.logical_and(heights > low_bound, heights < up_bound)] = color
            low_bound = up_bound
        return color_map

    def mark_points(self, image, pixels, colors):
        for dv in range(-1, 2):
            for du in range(-1, 2):
                v = np.clip(pixels[:, 1] + dv, 0, image.shape[0]-1)
                u = np.clip(pixels[:, 0] + du, 0, image.shape[1]-1)
                image[v, u] = colors
        return image

    def visualize_label(self, image, label):
        for instance in label:
            image = cv2.rectangle(image, (instance[1], instance[2]), (instance[3], instance[4]), (0, 100, 255), 2)
            image = cv2.putText(image, instance[0] + "_" + str(instance[-1]), (instance[1], instance[2]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        return image

    def check_region(self, image, pcd):
        copied_image = image.copy()
        pcd = np.concatenate(pcd, axis=0)
        pixel = pcd[:, 3:5].astype(np.int32)
        copied_image[pixel[:, 1], pixel[:, 0]] = (0, 0, 255)
        cv2.imshow("outer_label_check", copied_image)
