import cv2


def main():
    i = 0
    cap = cv2.VideoCapture(4)
    # 5 = lidar
    while True:
        _, frame = cap.read()
        cv2.imshow("frame", frame)
        k = cv2.waitKey(1) & 0xff
        if k != cv2.waitKey(1) & 0xff:
            print(k)
        if k == 27:
            cv2.destroyAllWindows()
            cap.release()
            return 0
        elif k == 99:
            i += 1
            print(123123)
            cv2.imwrite(f"pic{i}.jpg", frame)


if __name__ == '__main__':
    main()