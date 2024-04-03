import config as cfg
from dataset import Dataset
from pcd_split import PcdSplitter
from plane_estim import PlaneEstim
from height_estim import HeightEstim
from visualizer import HeightVisualizer


def main():
    dataset = Dataset(cfg.DATASET)
    num_frames = dataset.num_frame()
    pcd_split = PcdSplitter(cfg.PCD_SPLIT)
    plane_estim = PlaneEstim(cfg.PLANE_ESTIM)
    height_estim = HeightEstim(cfg.HEIGHT_ESTIM)
    height_vis = HeightVisualizer()
    start_index = 0
    for idx in range(num_frames):
        # pcd: [x y z]
        idx = idx + start_index
        image, pcd, label = dataset.get_frame(idx)
        if image is None:
            print("image is None", idx)
            continue

        # pcd: [x y z u v]
        pcd, section_xy_border = pcd_split.split_pcd_into_sections(pcd)
        # print("pcd shapes", [sec_pcd.shape for row_pcd in pcd for sec_pcd in row_pcd])
        planes, label_out_pcd = plane_estim.fit_section_planes(pcd, label)
        # print("pcd shapes", [sec_pcd.shape for row_pcd in pcd for sec_pcd in row_pcd])
        # pcd: [x y z u v h]
        pcd, label = height_estim.estimate_heights(pcd, label, planes)
        height_vis.visualize(image, pcd, label, section_xy_border)
        # dataset.update_label(label)

if __name__ == '__main__':
    main()
