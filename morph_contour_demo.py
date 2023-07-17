import sys
import cv2
import numpy as np
import time

from queue import Queue


class Timer:
    def __init__(self):
        self.start = time.time()
        self.count = 0

    def inc(self):
        self.count += 1

    def elapsed(self):
        return self.count / (time.time() - self.start)
    


def morph_contour_loop(video_port, kernel_side, min_space_width, queue):
    kernel_size = (kernel_side, kernel_side)
    cap = cv2.VideoCapture(video_port)
    timer = Timer()
    while True:
        frame, contours, close_contour, best = contour_inner_loop(cap, kernel_size, min_space_width)
        cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)
        cv2.drawContours(frame, close_contour, -1, (255, 0, 0), 3)
        cv2.drawContours(frame, best, -1, (0, 0, 255), 3)

        # Display the resulting frame
        cv2.imshow('frame', frame)
        timer.inc()

        if queue.empty():
            queue.put(best)

        # Exit the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    fps = timer.elapsed()
    cap.release()
    cv2.destroyAllWindows()
    print("FPS:", fps)
    queue.put("QUIT")


def contour_inner_loop(cap, kernel_size, min_space_width):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))
    contours, hierarchy = find_contours(frame, kernel_size)
    close_contour = find_close_contour(contours, cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    clusters = find_contour_clusters(close_contour)
    best = best_contour_cluster(clusters, min_space_width)
    return frame, contours, close_contour, best


def find_contours(frame, kernel_size):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # From https://www.scaler.com/topics/contour-analysis-opencv/
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
    filtered = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, kernel)

    # Contour material from https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html
    ret, thresh = cv2.threshold(filtered, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    return contours, hierarchy


def find_close_contour(contours, height):
    best_xs = {}
    for contour in contours:
        for point in contour:
            if point[0][1] < height - 1 and (point[0][0] not in best_xs or point[0][1] > best_xs[point[0][0]]):
                best_xs[point[0][0]] = point[0][1]
    close_contour = np.empty((len(best_xs), 1, 2), dtype=contours[0].dtype)
    for i, (x, y) in enumerate(best_xs.items()):
        close_contour[i] = np.array([[x, y]])
    return close_contour


def farthest_x_y(contour):
    min_y_index = np.argmin(contour[:, :, 1])
    return contour[min_y_index][0]


def sorted_contour_list(close_contour):
    return [(pt[0][0], pt[0][1]) for pt in sorted(list(close_contour), key=lambda pt: (pt[0][1], pt[0][0]))]


def find_contour_clusters(close_contour):
    clusters = []
    cluster = []
    x_sorted = [(pt[0][0], pt[0][1]) for pt in sorted(list(close_contour), key=lambda pt: (pt[0,0], pt[0,1]))]
    for (x, y) in x_sorted:
        if len(cluster) == 0 or (cluster[-1][0] + 1 == x and abs(cluster[-1][1] - y) <= 1):
            cluster.append((x, y))
        else:
            clusters.append(cluster)
            cluster = [(x, y)]
    clusters.append(cluster)
    result = []
    for cluster in clusters:
        pts = np.empty((len(cluster), 1, 2), dtype=close_contour[0].dtype)
        for i in range(len(cluster)):
            pts[i] = cluster[i]
        result.append(pts)
    return result


def best_contour_cluster(contour_clusters, min_space_width):
    heights = [max([c[0, 1] for c in cluster]) for cluster in contour_clusters]
    min_i = None
    for i in range(len(contour_clusters)):
        if min_i is None or heights[i] < heights[min_i] and len(contour_clusters[i]) > min_space_width:
            min_i = i
    return contour_clusters[min_i]


def contour_x_bounds(contour):
    return np.min(contour[:,:,0]), np.max(contour[:,:,0])


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: morph_contour_demo.py video_port kernel_side min_space_width")
    else:
        queue = Queue()
        morph_contour_loop(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]), queue)

