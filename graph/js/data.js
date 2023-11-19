var data = [
  [0.30, 3.82, 78.12, 1.89, 1.10],
  [0.89, 15.86, 66.85, 3.28, 1.05],
  [1.82, -6.79, 72.11, 1.71, 1.01],
  [3.03, 15.17, 72.69, 4.02, 0.94],
  [4.47, -12.68, 72.12, 4.28, 0.86],
  [6.10, 19.79, 69.71, 6.37, 0.80],
  [7.84, 11.06, 69.95, 6.72, 0.75],
  [9.68, -0.15, 68.20, 8.23, 0.70],
  [11.58, -0.40, 71.93, 10.04, 0.66],
  [13.51, 10.54, 69.87, 12.04, 0.63],
  [15.46, -1.53, 69.09, 14.62, 0.60],
  [17.43, 32.88, 66.72, 16.59, 0.58],
  [19.41, 23.90, 72.06, 19.16, 0.55],
  [21.39, 8.54, 68.46, 20.97, 0.53],
  [23.39, 35.08, 70.42, 23.18, 0.52],
  [25.38, 27.55, 72.51, 25.82, 0.50],
  [27.38, 13.31, 70.02, 27.84, 0.48],
  [29.38, 33.96, 71.89, 29.92, 0.47],
  [31.39, 12.12, 73.00, 32.03, 0.46],
  [33.39, 32.63, 69.00, 34.14, 0.45],
  [35.39, 49.46, 68.66, 36.46, 0.44],
  [37.40, 39.57, 63.35, 38.81, 0.43],
  [39.40, 30.83, 73.65, 41.19, 0.42],
  [41.41, 34.46, 67.90, 43.63, 0.41],
  [43.41, 35.45, 67.39, 46.10, 0.40],
  [45.42, 59.99, 66.91, 48.20, 0.39],
  [47.42, 60.27, 72.89, 50.32, 0.38],
  [49.43, 55.78, 69.35, 52.56, 0.38],
  [51.43, 34.49, 77.43, 54.45, 0.37],
  [53.44, 46.24, 70.06, 57.04, 0.37],
  [55.44, 41.92, 68.44, 59.11, 0.36],
  [57.45, 67.50, 72.25, 61.29, 0.35],
  [59.45, 67.31, 69.91, 63.75, 0.35],
  [61.46, 71.98, 73.07, 65.91, 0.34],
  [63.46, 76.38, 68.93, 68.21, 0.34],
  [65.47, 73.35, 66.20, 70.39, 0.33],
  [67.47, 67.89, 70.91, 72.45, 0.33],
  [69.48, 66.63, 67.13, 74.67, 0.32],
  [71.48, 77.70, 70.75, 76.83, 0.32],
  [73.49, 60.37, 66.94, 78.97, 0.32],
  [75.51, 68.09, 65.66, 81.20, 0.31],
  [77.54, 68.51, 69.15, 83.39, 0.31],
  [79.56, 74.37, 69.47, 85.78, 0.31],
  [81.59, 64.88, 64.03, 87.98, 0.30],
  [83.62, 71.75, 67.46, 90.07, 0.30],
  [85.64, 71.80, 73.13, 92.21, 0.30],
  [87.67, 103.82, 66.49, 94.50, 0.29],
  [89.69, 88.07, 62.98, 96.70, 0.29],
  [91.72, 100.23, 65.39, 98.84, 0.29],
  [93.74, 97.88, 67.42, 101.03, 0.28],
  [95.77, 84.21, 70.97, 103.16, 0.28],
  [97.79, 104.55, 68.15, 105.38, 0.28],
  [99.82, 103.16, 71.87, 107.65, 0.28],
  [101.84, 104.81, 70.52, 109.88, 0.27],
  [103.86, 96.29, 71.83, 111.89, 0.27],
  [105.89, 110.60, 71.88, 113.99, 0.27],
  [107.92, 111.84, 65.89, 116.19, 0.27],
  [109.94, 113.92, 69.85, 118.49, 0.27],
  [111.97, 127.76, 71.21, 120.60, 0.26],
  [113.99, 90.46, 74.18, 122.74, 0.26],
  [116.02, 110.44, 71.40, 124.96, 0.26],
  [118.04, 136.80, 72.50, 127.09, 0.26],
  [120.07, 126.50, 65.22, 129.17, 0.26],
  [122.09, 133.97, 67.45, 131.23, 0.25],
  [124.12, 135.33, 71.28, 133.33, 0.25],
  [126.14, 110.20, 67.68, 135.32, 0.25],
  [128.17, 112.62, 70.03, 137.45, 0.25],
  [130.19, 122.23, 68.78, 139.61, 0.25],
  [132.22, 132.78, 69.10, 141.78, 0.25],
  [134.24, 131.98, 69.87, 143.92, 0.24],
  [136.27, 147.18, 72.47, 145.97, 0.24],
  [138.29, 133.98, 67.79, 148.09, 0.24],
  [140.32, 153.23, 69.10, 150.18, 0.24],
  [142.34, 135.80, 74.05, 152.23, 0.24],
  [144.37, 150.47, 70.04, 154.27, 0.24],
  [146.39, 152.64, 70.51, 156.40, 0.24],
  [148.42, 142.02, 67.48, 158.52, 0.23],
  [150.44, 143.96, 69.88, 160.65, 0.23],
  [152.47, 158.57, 70.61, 162.84, 0.23],
  [154.49, 153.80, 69.33, 164.88, 0.23],
  [156.52, 167.22, 66.69, 166.94, 0.23],
  [158.54, 153.21, 69.32, 169.04, 0.23],
  [160.57, 162.81, 66.99, 171.15, 0.23],
  [162.59, 152.02, 69.38, 173.24, 0.23],
  [164.62, 165.02, 71.91, 175.45, 0.23],
  [166.64, 164.43, 69.07, 177.64, 0.23],
  [168.67, 154.62, 71.19, 179.75, 0.22],
  [170.69, 182.69, 66.39, 181.76, 0.22],
  [172.72, 192.99, 67.05, 183.89, 0.22],
  [174.74, 180.32, 70.12, 186.06, 0.22],
  [176.77, 187.81, 67.81, 188.16, 0.22],
  [178.79, 156.16, 69.96, 190.14, 0.22],
  [180.82, 183.87, 69.38, 192.16, 0.22],
  [182.84, 170.61, 70.21, 194.27, 0.22],
  [184.87, 178.79, 74.05, 196.27, 0.22],
  [186.89, 193.91, 67.79, 198.31, 0.22],
  [188.92, 200.48, 66.25, 200.37, 0.22],
  [190.94, 190.95, 65.96, 202.46, 0.22],
  [192.97, 180.50, 70.83, 204.42, 0.22],
  [194.99, 182.21, 67.99, 206.50, 0.22],
  [197.01, 210.51, 72.73, 208.50, 0.21],
  [199.02, 180.21, 68.36, 210.55, 0.21],
  [201.03, 209.25, 70.30, 212.66, 0.21],
  [203.01, 221.19, 65.13, 214.79, 0.21],
  [204.98, 213.13, 68.82, 216.92, 0.21],
  [206.94, 228.03, 65.89, 218.98, 0.21],
  [208.88, 202.99, 65.42, 221.04, 0.21],
  [210.80, 203.30, 59.30, 223.13, 0.21],
  [212.70, 225.21, 63.68, 225.27, 0.21],
  [214.58, 221.96, 63.60, 227.60, 0.21],
  [216.45, 215.54, 64.98, 229.69, 0.21],
  [218.30, 242.80, 59.46, 231.82, 0.21],
  [220.13, 206.21, 60.65, 233.94, 0.21],
  [221.94, 228.51, 59.81, 235.99, 0.21],
  [223.73, 226.44, 62.42, 238.05, 0.21],
  [225.50, 226.53, 59.70, 240.09, 0.21],
  [227.25, 246.23, 52.55, 242.16, 0.21],
  [228.97, 231.56, 51.72, 244.21, 0.21],
  [230.68, 237.23, 54.18, 246.16, 0.21],
  [232.37, 239.74, 58.45, 248.11, 0.21],
  [234.04, 222.56, 54.50, 250.02, 0.21],
  [235.69, 240.82, 53.89, 251.78, 0.21],
  [237.32, 253.17, 49.77, 253.53, 0.21],
  [238.92, 231.86, 49.35, 255.31, 0.21],
  [240.50, 238.80, 52.27, 256.96, 0.21],
  [242.06, 252.66, 48.08, 258.56, 0.21],
  [243.60, 250.92, 51.72, 260.20, 0.21],
  [245.12, 237.24, 52.35, 261.70, 0.21],
  [246.61, 251.39, 51.28, 263.14, 0.21],
  [248.08, 262.82, 50.95, 264.51, 0.21],
  [249.53, 241.82, 48.41, 265.86, 0.21],
  [250.95, 235.56, 48.60, 267.17, 0.21],
  [252.35, 263.20, 46.51, 268.42, 0.21],
  [253.72, 259.79, 43.58, 269.55, 0.21],
  [255.07, 247.53, 45.60, 270.66, 0.21],
  [256.39, 272.22, 42.84, 271.69, 0.21],
  [257.69, 271.94, 41.69, 272.65, 0.21],
  [258.96, 267.17, 40.32, 273.55, 0.21],
  [260.20, 260.30, 41.15, 274.33, 0.21],
  [261.42, 251.92, 40.97, 275.10, 0.21],
  [262.61, 261.42, 40.52, 275.84, 0.21],
  [263.77, 254.20, 41.62, 276.36, 0.21],
  [264.90, 261.71, 39.23, 277.00, 0.21],
  [266.00, 274.89, 40.36, 277.48, 0.20],
  [267.07, 263.99, 41.09, 277.80, 0.20],
  [268.11, 265.43, 36.91, 278.07, 0.20],
  [269.12, 265.26, 35.25, 278.23, 0.20],
  [270.10, 278.21, 36.05, 278.37, 0.20],
  [271.04, 266.16, 33.88, 278.50, 0.20],
  [271.95, 281.93, 35.58, 278.41, 0.20],
  [272.83, 266.03, 32.12, 278.38, 0.20],
  [273.67, 271.31, 31.21, 278.20, 0.20],
  [274.47, 268.56, 33.19, 277.98, 0.20],
  [275.23, 274.04, 31.63, 277.61, 0.20],
  [275.96, 291.97, 31.47, 277.13, 0.20],
  [276.64, 254.30, 30.20, 276.65, 0.20],
  [277.28, 267.59, 29.78, 276.11, 0.20],
  [277.88, 289.43, 29.91, 275.42, 0.20],
  [278.43, 299.80, 27.60, 274.54, 0.20],
  [278.93, 270.17, 26.63, 273.66, 0.20],
  [279.38, 278.01, 26.57, 272.68, 0.20],
  [279.78, 260.66, 26.80, 271.66, 0.20],
  [280.13, 278.25, 25.62, 270.53, 0.20],
  [280.41, 294.19, 24.05, 269.28, 0.20],
  [280.64, 297.24, 24.64, 267.88, 0.20],
  [280.79, 287.48, 21.49, 266.37, 0.20],
  [280.88, 255.17, 22.35, 264.79, 0.20],
  [280.88, 286.26, 22.85, 263.10, 0.20],
  [280.80, 284.68, 20.33, 261.30, 0.20],
  [280.63, 272.98, 20.77, 259.26, 0.20],
  [280.37, 286.24, 18.57, 257.22, 0.20],
  [280.01, 295.58, 18.61, 255.05, 0.20],
  [279.55, 294.70, 18.63, 252.87, 0.20],
  [278.99, 275.10, 18.82, 250.51, 0.20],
  [278.35, 270.70, 16.13, 248.14, 0.20],
  [277.61, 264.10, 15.23, 245.61, 0.20],
  [276.80, 272.79, 14.90, 243.10, 0.20],
  [275.90, 285.72, 14.56, 240.64, 0.20],
  [274.94, 273.71, 14.24, 238.19, 0.20],
  [273.91, 276.31, 12.82, 235.60, 0.20],
  [272.82, 266.25, 12.34, 233.00, 0.20],
  [271.68, 267.16, 11.25, 230.38, 0.20],
  [270.49, 279.50, 11.21, 227.65, 0.20],
  [269.25, 262.66, 10.15, 224.83, 0.20],
  [267.97, 279.50, 9.10, 222.01, 0.20],
  [266.65, 254.11, 9.04, 219.18, 0.20],
  [265.30, 274.24, 8.53, 216.31, 0.20],
  [263.91, 262.10, 7.49, 213.40, 0.20],
  [262.49, 279.69, 6.43, 210.34, 0.20],
  [261.04, 238.52, 5.90, 207.23, 0.20],
  [259.56, 258.38, 5.49, 203.99, 0.20],
  [258.05, 254.94, 4.57, 200.80, 0.20],
  [256.52, 251.00, 3.54, 197.53, 0.20],
  [254.96, 257.95, 2.94, 194.20, 0.20],
  [253.37, 250.07, 2.41, 190.83, 0.20],
  [251.76, 225.84, 1.62, 187.40, 0.20],
  [250.13, 247.92, 0.94, 183.90, 0.20],
  [248.47, 254.22, 0.26, 180.28, 0.20],
  [246.79, 268.02, 0.00, 176.70, 0.20],
  [245.09, 239.79, 0.00, 173.11, 0.20],
  [243.38, 255.79, 0.00, 169.48, 0.20],
  [241.66, 244.29, 0.00, 165.88, 0.20],
  [239.93, 253.96, 0.00, 162.26, 0.20],
  [238.20, 267.58, 0.00, 158.61, 0.20],
  [236.47, 230.99, 0.00, 154.97, 0.20],
  [234.73, 238.05, 0.00, 151.26, 0.20],
  [232.99, 225.12, 0.00, 147.48, 0.20],
  [231.25, 214.86, 0.00, 143.75, 0.20],
  [229.51, 228.76, 0.00, 139.92, 0.20],
  [227.76, 234.00, 0.00, 136.10, 0.20],
  [226.02, 231.13, 0.00, 132.26, 0.20],
  [224.28, 231.46, 0.00, 128.36, 0.20],
  [222.54, 231.35, 0.00, 124.43, 0.20],
  [220.79, 214.86, 0.00, 120.66, 0.20],
  [219.05, 217.32, 0.00, 116.82, 0.20],
  [217.31, 217.44, 0.00, 112.86, 0.20],
  [215.56, 222.60, 0.00, 108.93, 0.20],
  [213.82, 228.13, 0.00, 104.92, 0.20],
  [212.08, 214.24, 0.00, 100.96, 0.20],
  [210.33, 191.48, 0.00, 97.00, 0.20],
  [208.59, 231.46, 0.00, 93.04, 0.20],
  [206.85, 210.04, 0.00, 89.10, 0.20],
  [205.10, 214.74, 0.00, 85.11, 0.20],
  [203.36, 199.11, 0.00, 81.13, 0.20],
  [201.62, 204.42, 0.00, 77.24, 0.20],
  [199.87, 195.03, 0.00, 73.35, 0.20],
  [198.13, 189.87, 0.00, 69.41, 0.20],
  [196.38, 202.68, 0.00, 65.55, 0.20],
  [194.64, 196.66, 0.00, 61.59, 0.20],
  [192.90, 190.53, 0.00, 57.69, 0.20],
  [191.15, 185.83, 0.00, 53.75, 0.20],
  [189.41, 170.64, 0.00, 49.93, 0.20],
  [187.67, 189.99, 0.00, 46.11, 0.20],
  [185.92, 195.21, 0.00, 42.29, 0.20],
  [184.18, 182.23, 0.00, 38.38, 0.20],
  [182.44, 187.61, 0.00, 34.57, 0.20],
  [180.69, 187.80, 0.00, 30.75, 0.20],
  [178.95, 178.34, 0.00, 26.88, 0.20],
  [177.21, 178.38, 0.00, 23.14, 0.20],
  [175.46, 179.29, 0.00, 19.32, 0.20],
  [173.72, 165.03, 0.00, 15.62, 0.20],
  [171.98, 175.00, 0.00, 11.92, 0.20],
  [170.23, 155.22, 0.00, 8.20, 0.20],
  [168.49, 175.70, 0.00, 4.52, 0.20],
  [166.74, 156.95, 0.00, 0.92, 0.20],
  [165.00, 179.34, 0.00, -2.73, 0.20],
  [163.26, 165.40, 0.00, -6.31, 0.20],
  [161.51, 180.45, 0.00, -9.89, 0.20],
  [159.77, 153.04, 0.00, -13.40, 0.20],
  [158.03, 152.69, 0.00, -16.88, 0.20],
  [156.28, 163.05, 0.00, -20.31, 0.20],
  [154.54, 154.23, 0.00, -23.71, 0.20],
  [152.80, 141.20, 0.00, -27.09, 0.20],
  [151.05, 141.22, 0.00, -30.54, 0.20],
  [149.31, 140.25, 0.00, -33.88, 0.20],
  [147.57, 138.34, 0.00, -37.22, 0.20],
  [145.82, 144.01, 0.00, -40.51, 0.20],
  [144.08, 146.81, 0.00, -43.77, 0.20],
  [142.34, 156.32, 0.00, -46.96, 0.20],
  [140.59, 140.17, 0.00, -50.13, 0.20],
  [138.85, 144.85, 0.00, -53.33, 0.20],
  [137.10, 144.14, 0.00, -56.44, 0.20],
  [135.36, 135.71, 0.00, -59.59, 0.20],
  [133.62, 147.40, 0.00, -62.67, 0.20],
  [131.87, 127.51, 0.00, -65.70, 0.20],
  [130.13, 134.89, 0.00, -68.71, 0.20],
  [128.39, 132.56, 0.00, -71.70, 0.20],
  [126.64, 132.87, 0.00, -74.53, 0.20],
  [124.90, 122.96, 0.00, -77.42, 0.20],
  [123.16, 117.45, 0.00, -80.34, 0.20],
  [121.41, 146.71, 0.00, -83.22, 0.20],
  [119.67, 124.42, 0.00, -85.99, 0.20],
  [117.93, 134.70, 0.00, -88.76, 0.20],
  [116.18, 116.97, 0.00, -91.59, 0.20],
  [114.44, 124.53, 0.00, -94.40, 0.20],
  [112.70, 116.17, 0.00, -97.08, 0.20],
  [110.95, 91.70, 0.00, -99.74, 0.20],
  [109.21, 89.56, 0.00, -102.41, 0.20],
  [107.46, 124.73, 0.00, -104.99, 0.20],
  [105.72, 90.79, 0.00, -107.60, 0.20],
  [103.98, 107.59, 0.00, -110.07, 0.20],
  [102.23, 111.96, 0.00, -112.61, 0.20],
  [100.49, 98.34, 0.00, -115.12, 0.20],
  [98.75, 94.67, 0.00, -117.58, 0.20],
  [97.00, 92.29, 0.00, -119.99, 0.20],
  [95.26, 97.45, 0.00, -122.40, 0.20],
  [93.52, 101.03, 0.00, -124.77, 0.20],
  [91.77, 88.87, 0.00, -127.11, 0.20],
  [90.03, 95.03, 0.00, -129.45, 0.20],
  [88.29, 97.07, 0.00, -131.73, 0.20],
  [86.54, 82.50, 0.00, -133.98, 0.20],
  [84.80, 96.18, 0.00, -136.16, 0.20],
  [83.06, 94.88, 0.00, -138.41, 0.20],
  [81.31, 83.63, 0.00, -140.52, 0.20],
  [79.57, 87.69, 0.00, -142.62, 0.20],
  [77.82, 85.47, 0.00, -144.76, 0.20],
  [76.08, 61.63, 0.00, -146.85, 0.20],
  [74.34, 83.86, 0.00, -148.96, 0.20],
  [72.59, 77.64, 0.00, -151.00, 0.20],
  [70.85, 74.90, 0.00, -153.07, 0.20],
  [69.11, 69.61, 0.00, -155.06, 0.20],
  [67.36, 80.40, 0.00, -157.05, 0.20],
  [65.62, 63.90, 0.00, -158.98, 0.20],
  [63.88, 74.50, 0.00, -160.95, 0.20],
  [62.13, 66.78, 0.00, -162.93, 0.20],
  [60.39, 71.93, 0.00, -164.84, 0.20],
  [58.65, 44.90, 0.00, -166.71, 0.20],
  [56.90, 43.40, 0.00, -168.51, 0.20],
  [55.16, 44.27, 0.00, -170.39, 0.20],
  [53.42, 57.80, 0.00, -172.21, 0.20],
  [51.67, 60.13, 0.00, -174.03, 0.20],
  [49.93, 38.82, 0.00, -175.78, 0.20],
  [48.18, 55.26, 0.00, -177.57, 0.20],
  [46.44, 51.90, 0.00, -179.30, 0.20],
  [44.70, 54.64, 0.00, -181.02, 0.20],
  [42.95, 41.68, 0.00, -182.72, 0.20],
  [41.21, 48.53, 0.00, -184.37, 0.20],
  [39.48, 46.48, 0.00, -186.11, 0.20],
  [37.75, 37.26, 0.00, -187.84, 0.20],
  [36.03, 20.63, 0.00, -189.49, 0.20],
  [34.30, 33.86, 0.00, -191.07, 0.20],
  [32.58, 30.83, 0.00, -192.64, 0.20],
  [30.85, 27.21, 0.00, -194.32, 0.20],
  [29.12, 26.61, 0.00, -195.93, 0.20],
  [27.40, 27.50, 0.00, -197.54, 0.20],
  [25.67, 51.20, 0.00, -199.20, 0.20],
  [23.94, 3.13, 0.00, -200.78, 0.20],
  [22.22, 33.72, 0.00, -202.28, 0.20],
  [20.49, 20.77, 0.00, -203.83, 0.20],
  [18.77, 20.27, 0.00, -205.34, 0.20],
  [17.04, 20.58, 0.00, -206.84, 0.20],
  [15.31, 9.11, 0.00, -208.34, 0.20],
  [13.59, 14.08, 0.00, -209.86, 0.20],
  [11.86, 21.43, 0.00, -211.33, 0.20],
  [10.13, -12.61, 0.00, -212.77, 0.20],
  [8.41, 18.06, 0.00, -214.25, 0.20],
  [6.68, 4.45, 0.00, -215.71, 0.20],
  [4.96, 14.67, 0.00, -217.15, 0.20],
  [3.23, 2.79, 0.00, -218.65, 0.20],
  [1.50, -4.43, 0.00, -220.12, 0.20],
];
