var data = [
  [0.30, -13.08, 71.08, -1.38, 1.10],
  [0.89, 1.11, 71.76, -0.39, 1.05],
  [1.82, 8.87, 69.93, -0.06, 1.01],
  [3.03, 6.46, 64.57, 1.25, 0.94],
  [4.47, -2.83, 73.87, 3.21, 0.86],
  [6.09, -14.39, 69.95, 5.61, 0.80],
  [7.84, 4.76, 71.62, 7.86, 0.74],
  [9.68, -7.62, 67.90, 9.89, 0.70],
  [11.58, 8.04, 69.64, 12.19, 0.66],
  [13.51, 13.31, 65.32, 13.71, 0.63],
  [15.46, 11.97, 67.77, 15.95, 0.60],
  [17.43, 2.57, 64.83, 18.02, 0.58],
  [19.41, 25.08, 73.24, 20.21, 0.55],
  [21.39, 13.85, 75.82, 22.48, 0.53],
  [23.39, 15.64, 66.83, 24.07, 0.52],
  [25.39, 28.85, 70.31, 26.48, 0.50],
  [27.39, 26.76, 69.67, 28.35, 0.48],
  [29.39, 34.00, 70.11, 30.62, 0.47],
  [31.39, 21.19, 67.29, 32.37, 0.46],
  [33.39, 10.30, 68.48, 34.08, 0.45],
  [35.40, 18.44, 70.10, 36.38, 0.44],
  [37.40, 34.38, 68.68, 38.48, 0.43],
  [39.41, 15.23, 69.50, 40.58, 0.42],
  [41.41, 58.35, 73.88, 42.52, 0.41],
  [43.41, 27.72, 76.71, 44.61, 0.40],
  [45.42, 32.82, 67.25, 46.80, 0.39],
  [47.42, 33.36, 71.17, 48.69, 0.38],
  [49.43, 41.87, 73.35, 50.80, 0.38],
  [51.43, 40.37, 72.01, 52.93, 0.37],
  [53.44, 54.12, 66.40, 54.88, 0.37],
  [55.44, 71.26, 68.07, 56.87, 0.36],
  [57.45, 54.08, 67.70, 58.98, 0.35],
  [59.45, 62.99, 64.54, 60.97, 0.35],
  [61.46, 64.88, 68.21, 62.82, 0.34],
  [63.46, 73.65, 71.32, 64.70, 0.34],
  [65.47, 52.75, 73.92, 66.77, 0.33],
  [67.47, 69.60, 70.87, 68.67, 0.33],
  [69.48, 73.26, 74.33, 70.59, 0.32],
  [71.48, 63.69, 69.96, 72.66, 0.32],
  [73.49, 71.47, 71.67, 74.70, 0.32],
  [75.51, 68.74, 68.78, 76.69, 0.31],
  [77.54, 81.37, 74.85, 78.66, 0.31],
  [79.56, 80.04, 71.81, 80.62, 0.31],
  [81.59, 83.83, 71.43, 82.62, 0.30],
  [83.61, 90.42, 68.05, 84.38, 0.30],
  [85.63, 83.03, 64.04, 86.13, 0.30],
  [87.66, 88.09, 58.26, 88.13, 0.29],
  [89.69, 76.35, 70.28, 90.17, 0.29],
  [91.71, 99.81, 68.11, 92.24, 0.29],
  [93.74, 101.76, 73.68, 94.11, 0.28],
  [95.76, 94.40, 67.17, 96.07, 0.28],
  [97.79, 103.54, 68.76, 98.02, 0.28],
  [99.81, 107.28, 74.96, 100.13, 0.28],
  [101.84, 97.31, 70.55, 102.12, 0.27],
  [103.86, 112.58, 60.98, 104.10, 0.27],
  [105.89, 104.15, 67.28, 106.28, 0.27],
  [107.91, 124.40, 72.55, 108.31, 0.27],
  [109.94, 123.95, 66.37, 110.34, 0.27],
  [111.96, 107.91, 73.57, 112.32, 0.26],
  [113.99, 123.91, 73.34, 114.34, 0.26],
  [116.01, 128.33, 73.67, 116.41, 0.26],
  [118.04, 132.72, 72.43, 118.49, 0.26],
  [120.06, 140.21, 71.56, 120.46, 0.26],
  [122.09, 123.09, 72.21, 122.49, 0.25],
  [124.11, 130.47, 70.49, 124.51, 0.25],
  [126.14, 111.19, 75.20, 126.56, 0.25],
  [128.16, 106.86, 69.98, 128.43, 0.25],
  [130.19, 132.65, 70.04, 130.37, 0.25],
  [132.21, 143.32, 74.53, 132.44, 0.25],
  [134.24, 128.14, 61.65, 134.42, 0.25],
  [136.26, 117.41, 75.86, 136.52, 0.24],
  [138.29, 153.79, 74.63, 138.53, 0.24],
  [140.31, 134.87, 67.25, 140.56, 0.24],
  [142.34, 142.58, 71.50, 142.58, 0.24],
  [144.36, 139.07, 67.39, 144.56, 0.24],
  [146.39, 151.31, 63.18, 146.56, 0.24],
  [148.41, 158.10, 64.26, 148.55, 0.24],
  [150.44, 151.55, 74.56, 150.43, 0.24],
  [152.46, 149.53, 74.27, 152.55, 0.23],
  [154.49, 151.89, 67.03, 154.56, 0.23],
  [156.51, 154.73, 64.01, 156.56, 0.23],
  [158.54, 159.87, 66.93, 158.52, 0.23],
  [160.56, 163.27, 72.01, 160.45, 0.23],
  [162.59, 159.02, 71.01, 162.39, 0.23],
  [164.61, 157.72, 69.50, 164.39, 0.23],
  [166.64, 151.97, 70.44, 166.41, 0.23],
  [168.66, 167.44, 67.07, 168.38, 0.23],
  [170.69, 181.67, 70.01, 170.44, 0.23],
  [172.71, 184.65, 69.19, 172.47, 0.23],
  [174.74, 176.06, 68.02, 174.50, 0.22],
  [176.76, 197.57, 71.94, 176.51, 0.22],
  [178.79, 165.56, 74.86, 178.48, 0.22],
  [180.81, 183.50, 73.63, 180.51, 0.22],
  [182.84, 187.58, 75.15, 182.46, 0.22],
  [184.86, 200.54, 68.33, 184.60, 0.22],
  [186.89, 194.58, 74.09, 186.61, 0.22],
  [188.91, 196.46, 68.43, 188.69, 0.22],
  [190.94, 210.87, 71.37, 190.77, 0.22],
  [192.96, 189.05, 71.95, 192.77, 0.22],
  [194.99, 193.87, 65.54, 194.83, 0.22],
  [197.01, 192.53, 67.22, 196.81, 0.22],
  [199.02, 190.35, 67.09, 198.83, 0.22],
  [201.02, 205.36, 67.07, 200.84, 0.22],
  [203.01, 197.31, 68.96, 202.82, 0.22],
  [204.98, 175.38, 66.57, 204.86, 0.22],
  [206.93, 214.70, 67.20, 206.85, 0.22],
  [208.87, 194.49, 71.24, 208.77, 0.22],
  [210.79, 228.45, 67.62, 210.69, 0.22],
  [212.70, 208.20, 66.36, 212.59, 0.22],
  [214.58, 208.53, 68.20, 214.47, 0.22],
  [216.45, 209.57, 62.18, 216.38, 0.22],
  [218.29, 220.54, 62.11, 218.26, 0.21],
  [220.12, 226.75, 63.02, 220.12, 0.21],
  [221.93, 209.70, 62.25, 222.01, 0.21],
  [223.72, 218.64, 57.16, 223.79, 0.21],
  [225.49, 215.75, 61.13, 225.58, 0.21],
  [227.24, 213.93, 58.52, 227.41, 0.21],
  [228.97, 240.06, 59.34, 229.18, 0.21],
  [230.68, 230.42, 56.54, 230.90, 0.21],
  [232.37, 217.88, 61.41, 232.61, 0.21],
  [234.04, 228.61, 53.26, 234.23, 0.21],
  [235.69, 231.82, 50.72, 235.96, 0.21],
  [237.32, 240.12, 52.98, 237.60, 0.21],
  [238.92, 250.14, 48.63, 239.15, 0.21],
  [240.50, 252.14, 58.01, 240.84, 0.21],
  [242.06, 248.58, 52.11, 242.45, 0.21],
  [243.60, 235.44, 49.44, 244.05, 0.21],
  [245.12, 241.10, 51.00, 245.54, 0.21],
  [246.61, 241.70, 46.46, 247.00, 0.21],
  [248.08, 255.70, 43.90, 248.45, 0.21],
  [249.53, 232.56, 46.33, 249.88, 0.21],
  [250.95, 235.16, 48.41, 251.25, 0.21],
  [252.35, 270.74, 45.18, 252.66, 0.21],
  [253.72, 264.40, 50.14, 254.15, 0.21],
  [255.07, 252.16, 48.62, 255.52, 0.21],
  [256.39, 243.32, 43.32, 256.79, 0.21],
  [257.69, 258.68, 42.03, 258.02, 0.21],
  [258.96, 251.15, 39.95, 259.32, 0.21],
  [260.20, 259.33, 36.53, 260.62, 0.21],
  [261.42, 268.48, 41.27, 261.83, 0.21],
  [262.61, 252.15, 38.39, 262.98, 0.21],
  [263.77, 262.59, 42.42, 264.16, 0.21],
  [264.90, 269.31, 38.69, 265.27, 0.21],
  [266.00, 266.74, 39.32, 266.43, 0.21],
  [267.08, 284.15, 37.27, 267.52, 0.21],
  [268.12, 273.26, 34.75, 268.47, 0.21],
  [269.13, 261.64, 33.18, 269.39, 0.21],
  [270.10, 276.21, 37.94, 270.32, 0.21],
  [271.05, 258.12, 36.71, 271.32, 0.21],
  [271.96, 262.72, 34.41, 272.24, 0.21],
  [272.83, 257.03, 33.95, 273.12, 0.21],
  [273.67, 276.26, 34.33, 273.97, 0.21],
  [274.47, 277.74, 30.85, 274.75, 0.21],
  [275.24, 267.06, 32.14, 275.42, 0.21],
  [275.96, 273.19, 32.54, 276.12, 0.21],
  [276.64, 284.23, 30.65, 276.78, 0.21],
  [277.29, 275.29, 31.12, 277.41, 0.21],
  [277.88, 268.55, 30.08, 277.94, 0.21],
  [278.43, 267.96, 26.93, 278.50, 0.21],
  [278.94, 297.69, 26.14, 278.98, 0.21],
  [279.39, 284.54, 26.95, 279.46, 0.21],
  [279.79, 275.52, 26.77, 279.93, 0.21],
  [280.13, 276.87, 23.63, 280.27, 0.21],
  [280.42, 277.36, 24.04, 280.58, 0.21],
  [280.64, 285.57, 22.20, 280.82, 0.21],
  [280.80, 281.20, 21.64, 281.06, 0.21],
  [280.88, 289.64, 24.65, 281.14, 0.21],
  [280.89, 296.58, 23.11, 281.16, 0.21],
  [280.81, 260.46, 19.49, 281.03, 0.21],
  [280.64, 259.95, 19.74, 280.76, 0.21],
  [280.37, 270.65, 19.82, 280.45, 0.21],
  [280.01, 285.89, 19.62, 280.07, 0.21],
  [279.55, 271.25, 17.70, 279.60, 0.21],
  [279.00, 291.35, 16.75, 279.03, 0.21],
  [278.35, 271.59, 16.50, 278.45, 0.21],
  [277.62, 287.49, 15.72, 277.73, 0.21],
  [276.80, 301.21, 15.01, 277.01, 0.21],
  [275.91, 276.23, 15.43, 276.25, 0.21],
  [274.94, 275.52, 13.43, 275.37, 0.21],
  [273.91, 289.15, 13.07, 274.37, 0.21],
  [272.83, 250.70, 12.79, 273.44, 0.21],
  [271.68, 272.62, 12.17, 272.38, 0.21],
  [270.49, 264.67, 11.04, 271.22, 0.21],
  [269.25, 273.64, 10.84, 270.07, 0.21],
  [267.97, 261.22, 9.58, 268.89, 0.21],
  [266.65, 267.05, 8.68, 267.63, 0.21],
  [265.30, 253.44, 7.34, 266.33, 0.21],
  [263.91, 255.77, 6.52, 265.05, 0.21],
  [262.49, 264.93, 6.89, 263.71, 0.21],
  [261.04, 275.22, 5.89, 262.29, 0.21],
  [259.56, 264.08, 5.17, 260.80, 0.21],
  [258.05, 249.77, 4.49, 259.32, 0.21],
  [256.52, 256.21, 3.90, 257.82, 0.21],
  [254.96, 254.81, 3.11, 256.25, 0.21],
  [253.37, 254.13, 2.36, 254.67, 0.21],
  [251.76, 244.23, 1.68, 253.07, 0.21],
  [250.13, 236.99, 0.92, 251.41, 0.21],
  [248.47, 257.41, 0.24, 249.79, 0.21],
  [246.79, 251.94, 0.00, 248.12, 0.21],
  [245.09, 258.58, 0.00, 246.39, 0.21],
  [243.38, 250.82, 0.00, 244.78, 0.21],
  [241.66, 255.24, 0.00, 243.02, 0.21],
  [239.93, 255.70, 0.00, 241.35, 0.21],
  [238.20, 233.57, 0.00, 239.63, 0.21],
  [236.47, 233.57, 0.00, 237.90, 0.21],
  [234.73, 239.36, 0.00, 236.16, 0.21],
  [232.99, 246.63, 0.00, 234.40, 0.21],
  [231.25, 218.14, 0.00, 232.71, 0.21],
  [229.51, 235.18, 0.00, 230.96, 0.21],
  [227.76, 229.91, 0.00, 229.19, 0.21],
  [226.02, 208.33, 0.00, 227.47, 0.21],
  [224.28, 235.92, 0.00, 225.73, 0.21],
  [222.54, 222.21, 0.00, 223.98, 0.21],
  [220.79, 206.77, 0.00, 222.19, 0.21],
  [219.05, 226.73, 0.00, 220.42, 0.21],
  [217.31, 223.61, 0.00, 218.63, 0.21],
  [215.56, 215.29, 0.00, 216.85, 0.21],
  [213.82, 217.74, 0.00, 215.14, 0.21],
  [212.07, 215.63, 0.00, 213.38, 0.21],
  [210.33, 228.12, 0.00, 211.57, 0.21],
  [208.59, 201.49, 0.00, 209.80, 0.21],
  [206.84, 193.86, 0.00, 207.98, 0.21],
  [205.10, 206.46, 0.00, 206.24, 0.21],
  [203.36, 196.88, 0.00, 204.51, 0.21],
  [201.61, 202.44, 0.00, 202.68, 0.21],
  [199.87, 189.45, 0.00, 200.93, 0.21],
  [198.13, 211.12, 0.00, 199.17, 0.21],
  [196.38, 191.32, 0.00, 197.32, 0.21],
  [194.64, 202.34, 0.00, 195.54, 0.21],
  [192.90, 183.65, 0.00, 193.77, 0.21],
  [191.15, 194.94, 0.00, 191.94, 0.21],
  [189.41, 184.62, 0.00, 190.22, 0.21],
  [187.67, 186.91, 0.00, 188.44, 0.21],
  [185.92, 186.56, 0.00, 186.67, 0.21],
  [184.18, 186.52, 0.00, 184.84, 0.21],
  [182.44, 189.98, 0.00, 183.17, 0.21],
  [180.69, 189.26, 0.00, 181.37, 0.21],
  [178.95, 160.74, 0.00, 179.64, 0.21],
  [177.20, 180.51, 0.00, 177.86, 0.21],
  [175.46, 188.02, 0.00, 176.10, 0.21],
  [173.72, 172.62, 0.00, 174.34, 0.21],
  [171.97, 162.92, 0.00, 172.58, 0.21],
  [170.23, 169.36, 0.00, 170.80, 0.21],
  [168.49, 163.16, 0.00, 169.02, 0.21],
  [166.74, 159.01, 0.00, 167.31, 0.21],
  [165.00, 173.33, 0.00, 165.60, 0.21],
  [163.26, 167.82, 0.00, 163.75, 0.21],
  [161.51, 172.14, 0.00, 161.97, 0.21],
  [159.77, 153.18, 0.00, 160.22, 0.21],
  [158.03, 180.86, 0.00, 158.32, 0.21],
  [156.28, 147.40, 0.00, 156.58, 0.21],
  [154.54, 166.21, 0.00, 154.77, 0.21],
  [152.80, 148.08, 0.00, 153.00, 0.21],
  [151.05, 148.05, 0.00, 151.21, 0.21],
  [149.31, 151.70, 0.00, 149.49, 0.21],
  [147.56, 149.54, 0.00, 147.76, 0.21],
  [145.82, 148.44, 0.00, 146.04, 0.21],
  [144.08, 146.19, 0.00, 144.31, 0.21],
  [142.33, 147.17, 0.00, 142.55, 0.21],
  [140.59, 124.35, 0.00, 140.76, 0.21],
  [138.85, 142.09, 0.00, 139.02, 0.21],
  [137.10, 149.08, 0.00, 137.24, 0.21],
  [135.36, 138.88, 0.00, 135.52, 0.21],
  [133.62, 141.67, 0.00, 133.78, 0.21],
  [131.87, 141.42, 0.00, 132.02, 0.21],
  [130.13, 121.67, 0.00, 130.38, 0.21],
  [128.39, 128.18, 0.00, 128.55, 0.21],
  [126.64, 127.01, 0.00, 126.79, 0.21],
  [124.90, 137.82, 0.00, 125.04, 0.21],
  [123.16, 117.27, 0.00, 123.20, 0.21],
  [121.41, 115.85, 0.00, 121.44, 0.21],
  [119.67, 110.97, 0.00, 119.62, 0.21],
  [117.92, 102.58, 0.00, 117.91, 0.21],
  [116.18, 122.62, 0.00, 116.09, 0.21],
  [114.44, 115.95, 0.00, 114.32, 0.21],
  [112.69, 121.02, 0.00, 112.52, 0.21],
  [110.95, 93.45, 0.00, 110.85, 0.21],
  [109.21, 104.04, 0.00, 109.04, 0.21],
  [107.46, 116.68, 0.00, 107.34, 0.21],
  [105.72, 110.38, 0.00, 105.62, 0.21],
  [103.98, 113.67, 0.00, 103.90, 0.21],
  [102.23, 120.52, 0.00, 102.11, 0.21],
  [100.49, 105.04, 0.00, 100.39, 0.21],
  [98.75, 91.60, 0.00, 98.63, 0.21],
  [97.00, 112.96, 0.00, 96.87, 0.21],
  [95.26, 97.25, 0.00, 95.09, 0.21],
  [93.52, 85.93, 0.00, 93.26, 0.21],
  [91.77, 97.07, 0.00, 91.53, 0.21],
  [90.03, 90.12, 0.00, 89.75, 0.21],
  [88.28, 86.80, 0.00, 87.99, 0.21],
  [86.54, 79.68, 0.00, 86.29, 0.21],
  [84.80, 61.12, 0.00, 84.55, 0.21],
  [83.05, 73.83, 0.00, 82.81, 0.21],
  [81.31, 96.15, 0.00, 81.07, 0.21],
  [79.57, 80.85, 0.00, 79.30, 0.21],
  [77.82, 85.23, 0.00, 77.63, 0.21],
  [76.08, 77.88, 0.00, 75.87, 0.21],
  [74.34, 76.24, 0.00, 74.12, 0.21],
  [72.59, 71.97, 0.00, 72.37, 0.21],
  [70.85, 81.51, 0.00, 70.60, 0.21],
  [69.11, 53.55, 0.00, 68.82, 0.21],
  [67.36, 66.74, 0.00, 67.16, 0.21],
  [65.62, 77.38, 0.00, 65.48, 0.21],
  [63.88, 69.19, 0.00, 63.66, 0.21],
  [62.13, 42.47, 0.00, 61.88, 0.21],
  [60.39, 77.41, 0.00, 60.16, 0.21],
  [58.64, 61.20, 0.00, 58.40, 0.21],
  [56.90, 47.02, 0.00, 56.63, 0.21],
  [55.16, 50.90, 0.00, 54.91, 0.21],
  [53.41, 53.56, 0.00, 53.16, 0.21],
  [51.67, 52.09, 0.00, 51.46, 0.21],
  [49.93, 69.56, 0.00, 49.73, 0.21],
  [48.18, 51.40, 0.00, 48.08, 0.21],
  [46.44, 46.98, 0.00, 46.37, 0.21],
  [44.70, 32.85, 0.00, 44.66, 0.21],
  [42.95, 35.98, 0.00, 42.92, 0.21],
  [41.21, 34.11, 0.00, 41.23, 0.21],
  [39.48, 11.46, 0.00, 39.45, 0.21],
  [37.75, 45.18, 0.00, 37.70, 0.21],
  [36.03, 46.31, 0.00, 35.88, 0.21],
  [34.30, 28.51, 0.00, 34.18, 0.21],
  [32.57, 40.26, 0.00, 32.44, 0.21],
  [30.85, 27.14, 0.00, 30.68, 0.21],
  [29.12, 34.14, 0.00, 28.92, 0.21],
  [27.40, 34.80, 0.00, 27.17, 0.21],
  [25.67, 17.52, 0.00, 25.47, 0.21],
  [23.94, 25.98, 0.00, 23.67, 0.21],
  [22.22, 23.52, 0.00, 21.82, 0.21],
  [20.49, 39.96, 0.00, 20.13, 0.21],
  [18.76, 17.14, 0.00, 18.40, 0.21],
  [17.04, 32.76, 0.00, 16.64, 0.21],
  [15.31, 27.79, 0.00, 14.89, 0.21],
  [13.59, 0.77, 0.00, 13.18, 0.21],
  [11.86, 8.61, 0.00, 11.39, 0.21],
  [10.13, 6.70, 0.00, 9.72, 0.21],
  [8.41, 3.83, 0.00, 7.93, 0.21],
  [6.68, 0.12, 0.00, 6.30, 0.21],
  [4.95, -13.19, 0.00, 4.53, 0.21],
  [3.23, 7.54, 0.00, 2.81, 0.21],
  [1.50, 4.19, 0.00, 1.11, 0.21],
];
