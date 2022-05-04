import json
import linecache
import matplotlib.pyplot as plt

PATH_TO_FOLDER = "/home/maks/catkin_ws/src/diplom/records/test.json"

if __name__ == '__main__':
    with open(PATH_TO_FOLDER, 'r') as jsonfile:
        lines = jsonfile.readlines()
        b = json.loads(lines[0])

        #dist_hist = b["dist"]
        #(n, bins, patches) = plt.hist(dist_hist, bins = 100, range = (0.1, 200))
        #print(n, bins, patches)

        data_hist = b["hist"]
        #plt.hist(data_hist, bins = 50, range = (0, 50))
        plt.hist(data_hist, bins = 25)
        plt.show()


# TEMP
#data = """
#{
#    "pos": [0, 0, 0],
#    "dist": (0.1, 0.2, 0.3, 0.4, 0.5)
#}
#"""
#data_dict = json.loads(data)