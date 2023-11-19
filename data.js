var data = [
  [0.39, 0.84, 59.94, 0.35, 0.33],
  [1.22, 1.75, 59.92, 1.29, 0.24],
  [2.52, 4.31, 60.11, 2.68, 0.2],
  [4.18, 5.31, 60.05, 4.26, 0.18],
  [6.09, 6.93, 60.07, 6.01, 0.17],
  [8.19, 6.91, 60.02, 7.82, 0.16],
  [10.4, 11.26, 60.15, 9.95, 0.15],
  [12.69, 11.22, 59.98, 12.2, 0.14],
  [15.02, 14.88, 59.95, 14.46, 0.14],
  [17.39, 18.26, 60.09, 16.8, 0.13],
  [19.77, 18.94, 59.88, 19.08, 0.13],
  [22.16, 21.16, 59.94, 21.46, 0.12],
  [24.56, 24.48, 59.89, 24.0, 0.12],
  [26.97, 25.45, 59.92, 26.43, 0.12],
  [29.38, 31.77, 59.97, 29.02, 0.11],
  [31.79, 33.91, 59.84, 31.52, 0.11],
  [34.2, 32.35, 60.05, 34.05, 0.11],
  [36.61, 37.9, 60.06, 36.58, 0.11],
  [39.02, 38.88, 60.09, 39.15, 0.1],
  [41.43, 42.45, 59.93, 41.7, 0.1],
  [43.84, 45.14, 59.38, 44.26, 0.1],
  [46.25, 47.5, 58.91, 46.77, 0.1],
  [48.65, 47.62, 58.25, 49.28, 0.1],
  [51.04, 51.67, 57.65, 51.74, 0.09],
  [53.42, 52.59, 57.18, 54.2, 0.09],
  [55.79, 54.72, 56.25, 56.64, 0.09],
  [58.14, 60.15, 55.78, 59.11, 0.09],
  [60.49, 59.89, 55.09, 61.52, 0.09],
  [62.82, 64.89, 54.61, 63.96, 0.09],
  [65.14, 67.12, 54.01, 66.39, 0.09],
  [67.44, 65.86, 53.29, 68.77, 0.09],
  [69.74, 69.67, 52.81, 71.13, 0.09],
  [72.02, 72.88, 52.15, 73.48, 0.08],
  [74.28, 74.79, 51.76, 75.82, 0.08],
  [76.54, 78.64, 50.92, 78.16, 0.08],
  [78.78, 79.78, 50.53, 80.41, 0.08],
  [81.01, 80.77, 49.85, 82.68, 0.08],
  [83.22, 82.06, 49.11, 84.91, 0.08],
  [85.42, 84.5, 48.53, 87.11, 0.08],
  [87.61, 88.25, 48.05, 89.33, 0.08],
  [89.78, 89.79, 47.31, 91.51, 0.08],
  [91.95, 92.87, 46.76, 93.69, 0.08],
  [94.09, 94.26, 46.13, 95.85, 0.08],
  [96.23, 94.52, 45.64, 97.98, 0.08],
  [98.35, 100.94, 45.01, 100.1, 0.08],
  [100.46, 98.56, 44.28, 102.22, 0.07],
  [102.55, 101.55, 43.8, 104.3, 0.07],
  [104.63, 103.16, 43.29, 106.35, 0.07],
  [106.69, 107.47, 42.53, 108.4, 0.07],
  [108.74, 108.85, 42.02, 110.43, 0.07],
  [110.78, 112.01, 41.4, 112.48, 0.07],
  [112.8, 115.29, 40.83, 114.52, 0.07],
  [114.8, 114.9, 40.19, 116.53, 0.07],
  [116.8, 116.71, 39.65, 118.48, 0.07],
  [118.77, 118.88, 39.04, 120.45, 0.07],
  [120.73, 119.37, 38.39, 122.37, 0.07],
  [122.68, 122.94, 37.71, 124.3, 0.07],
  [124.61, 125.21, 37.17, 126.22, 0.07],
  [126.53, 127.39, 36.52, 128.13, 0.07],
  [128.43, 127.8, 36.03, 130.0, 0.07],
  [130.31, 131.8, 35.46, 131.85, 0.07],
  [132.18, 132.84, 34.68, 133.71, 0.07],
  [134.03, 135.3, 34.06, 135.55, 0.07],
  [135.87, 135.01, 33.59, 137.37, 0.07],
  [137.69, 137.34, 32.98, 139.18, 0.07],
  [139.5, 139.25, 32.39, 140.95, 0.07],
  [141.28, 141.4, 31.86, 142.7, 0.07],
  [143.06, 142.39, 31.21, 144.43, 0.07],
  [144.81, 144.82, 30.62, 146.13, 0.07],
  [146.55, 144.87, 29.9, 147.83, 0.07],
  [148.27, 147.98, 29.44, 149.53, 0.07],
  [149.98, 148.85, 28.75, 151.17, 0.07],
  [151.66, 152.71, 28.24, 152.81, 0.07],
  [153.33, 154.26, 27.59, 154.44, 0.07],
  [154.98, 155.14, 26.99, 156.05, 0.07],
  [156.62, 156.68, 26.4, 157.64, 0.07],
  [158.24, 158.19, 25.79, 159.22, 0.07],
  [159.83, 161.3, 25.17, 160.78, 0.07],
  [161.41, 163.24, 24.48, 162.3, 0.07],
  [162.97, 162.27, 23.98, 163.81, 0.07],
  [164.52, 163.93, 23.39, 165.31, 0.07],
  [166.04, 164.65, 22.78, 166.79, 0.07],
  [167.55, 166.26, 22.12, 168.23, 0.07],
  [169.03, 167.05, 21.61, 169.66, 0.07],
  [170.5, 172.34, 20.99, 171.1, 0.07],
  [171.94, 172.37, 20.4, 172.49, 0.07],
  [173.37, 173.49, 19.81, 173.88, 0.07],
  [174.77, 174.12, 19.1, 175.23, 0.07],
  [176.16, 176.88, 18.57, 176.6, 0.07],
  [177.52, 177.94, 17.97, 177.9, 0.07],
  [178.86, 179.69, 17.32, 179.2, 0.07],
  [180.19, 180.67, 16.75, 180.48, 0.07],
  [181.49, 180.63, 16.13, 181.72, 0.07],
  [182.76, 182.37, 15.56, 182.93, 0.07],
  [184.02, 181.73, 14.97, 184.13, 0.07],
  [185.25, 186.2, 14.31, 185.32, 0.07],
  [186.46, 187.03, 13.77, 186.46, 0.07],
  [187.64, 187.71, 13.12, 187.59, 0.07],
  [188.81, 189.83, 12.58, 188.71, 0.07],
  [189.94, 189.41, 11.96, 189.79, 0.07],
  [191.06, 189.68, 11.38, 190.85, 0.07],
  [192.14, 191.23, 10.75, 191.9, 0.07],
  [193.21, 192.83, 10.17, 192.89, 0.07],
  [194.24, 194.33, 9.55, 193.89, 0.07],
  [195.25, 194.47, 8.98, 194.88, 0.07],
  [196.23, 196.6, 8.37, 195.85, 0.07],
  [197.18, 197.0, 7.78, 196.76, 0.07],
  [198.11, 195.12, 7.16, 197.62, 0.07],
  [199.0, 198.88, 6.56, 198.46, 0.07],
  [199.87, 199.93, 5.95, 199.31, 0.07],
  [200.71, 202.56, 5.35, 200.1, 0.07],
  [201.51, 202.18, 4.75, 200.87, 0.07],
  [202.28, 203.23, 4.15, 201.6, 0.07],
  [203.02, 203.97, 3.56, 202.31, 0.07],
  [203.72, 205.26, 2.95, 202.97, 0.07],
  [204.39, 204.36, 2.35, 203.6, 0.07],
  [205.02, 204.41, 1.75, 204.2, 0.07],
  [205.61, 204.93, 1.15, 204.75, 0.07],
  [206.17, 206.72, 0.55, 205.27, 0.07],
  [206.68, 206.99, 0.0, 205.74, 0.07],
  [207.15, 207.31, 0.0, 206.2, 0.07],
  [207.58, 208.25, 0.0, 206.63, 0.07],
  [207.98, 208.17, 0.0, 207.03, 0.07],
  [208.35, 206.91, 0.0, 207.4, 0.07],
  [208.68, 209.94, 0.0, 207.75, 0.07],
  [208.99, 209.74, 0.0, 208.09, 0.07],
  [209.27, 208.31, 0.0, 208.37, 0.07],
  [209.52, 210.26, 0.0, 208.65, 0.07],
  [209.75, 209.29, 0.0, 208.9, 0.07],
  [209.96, 208.85, 0.0, 209.09, 0.07],
  [210.14, 210.61, 0.0, 209.25, 0.07],
  [210.31, 209.21, 0.0, 209.41, 0.07],
  [210.45, 211.4, 0.0, 209.57, 0.07],
  [210.58, 212.08, 0.0, 209.72, 0.07],
  [210.68, 211.62, 0.0, 209.82, 0.07],
  [210.77, 210.42, 0.0, 209.91, 0.07],
  [210.84, 212.58, 0.0, 209.98, 0.07],
  [210.89, 211.47, 0.0, 210.03, 0.07],
  [210.92, 211.65, 0.0, 210.04, 0.07],
  [210.93, 211.23, 0.0, 210.05, 0.07],
  [210.92, 211.7, 0.0, 210.04, 0.07],
  [210.9, 211.67, 0.0, 210.03, 0.07],
  [210.86, 211.44, 0.0, 209.99, 0.07],
  [210.8, 211.15, 0.0, 209.94, 0.07],
  [210.72, 210.08, 0.0, 209.84, 0.07],
  [210.63, 209.96, 0.0, 209.73, 0.07],
  [210.52, 210.46, 0.0, 209.62, 0.07],
  [210.39, 209.54, 0.0, 209.5, 0.07],
  [210.25, 209.84, 0.0, 209.37, 0.07],
  [210.09, 209.46, 0.0, 209.22, 0.07],
  [209.92, 209.21, 0.0, 209.06, 0.07],
  [209.73, 209.72, 0.0, 208.88, 0.07],
  [209.52, 210.49, 0.0, 208.69, 0.07],
  [209.31, 210.54, 0.0, 208.49, 0.07],
  [209.08, 209.05, 0.0, 208.28, 0.07],
  [208.83, 208.72, 0.0, 208.06, 0.07],
  [208.58, 208.57, 0.0, 207.83, 0.07],
  [208.31, 208.18, 0.0, 207.57, 0.07],
  [208.04, 207.08, 0.0, 207.31, 0.07],
  [207.75, 206.5, 0.0, 207.05, 0.07],
  [207.45, 206.62, 0.0, 206.78, 0.07],
  [207.15, 206.48, 0.0, 206.48, 0.07],
  [206.84, 206.23, 0.0, 206.17, 0.07],
  [206.51, 206.15, 0.0, 205.84, 0.07],
  [206.18, 206.8, 0.0, 205.54, 0.07],
  [205.85, 204.63, 0.0, 205.2, 0.07],
  [205.51, 205.66, 0.0, 204.86, 0.07],
  [205.16, 203.98, 0.0, 204.52, 0.07],
  [204.8, 203.74, 0.0, 204.18, 0.07],
  [204.44, 206.36, 0.0, 203.81, 0.07],
  [204.08, 204.9, 0.0, 203.47, 0.07],
  [203.71, 204.85, 0.0, 203.12, 0.07],
  [203.34, 202.17, 0.0, 202.76, 0.07],
  [202.96, 204.11, 0.0, 202.37, 0.07],
  [202.58, 201.72, 0.0, 201.98, 0.07],
  [202.2, 202.84, 0.0, 201.62, 0.07],
  [201.81, 199.8, 0.0, 201.27, 0.07],
  [201.42, 199.97, 0.0, 200.89, 0.07],
  [201.02, 202.23, 0.0, 200.5, 0.07],
  [200.63, 201.51, 0.0, 200.12, 0.07],
  [200.23, 200.08, 0.0, 199.72, 0.07],
  [199.83, 201.33, 0.0, 199.32, 0.07],
  [199.43, 199.07, 0.0, 198.91, 0.07],
  [199.03, 199.85, 0.0, 198.52, 0.07],
  [198.62, 197.57, 0.0, 198.08, 0.07],
  [198.21, 198.59, 0.0, 197.66, 0.07],
  [197.8, 197.34, 0.0, 197.25, 0.07],
  [197.39, 197.61, 0.0, 196.85, 0.07],
  [196.98, 196.51, 0.0, 196.43, 0.07],
  [196.57, 198.19, 0.0, 196.03, 0.07],
  [196.16, 196.22, 0.0, 195.62, 0.07],
  [195.75, 196.73, 0.0, 195.2, 0.07],
  [195.33, 196.24, 0.0, 194.79, 0.07],
  [194.91, 195.49, 0.0, 194.38, 0.07],
  [194.5, 193.83, 0.0, 193.98, 0.07],
  [194.08, 194.94, 0.0, 193.61, 0.07],
  [193.66, 194.98, 0.0, 193.21, 0.07],
  [193.25, 192.05, 0.0, 192.79, 0.07],
  [192.83, 192.65, 0.0, 192.39, 0.07],
  [192.41, 193.07, 0.0, 192.0, 0.07],
  [191.99, 191.77, 0.0, 191.57, 0.07],
  [191.57, 190.46, 0.0, 191.15, 0.07],
  [191.15, 190.3, 0.0, 190.73, 0.07],
  [190.73, 190.51, 0.0, 190.32, 0.07],
  [190.31, 189.9, 0.0, 189.92, 0.07],
  [189.88, 189.06, 0.0, 189.48, 0.07],
  [189.46, 189.24, 0.0, 189.05, 0.07],
  [189.04, 187.77, 0.0, 188.64, 0.07],
  [188.62, 187.69, 0.0, 188.22, 0.07],
  [188.2, 185.4, 0.0, 187.79, 0.07],
  [187.77, 188.0, 0.0, 187.4, 0.07],
  [187.35, 186.46, 0.0, 186.98, 0.07],
  [186.93, 186.93, 0.0, 186.55, 0.07],
  [186.51, 185.92, 0.0, 186.17, 0.07],
  [186.08, 185.67, 0.0, 185.73, 0.07],
  [185.66, 186.28, 0.0, 185.29, 0.07],
  [185.24, 182.97, 0.0, 184.87, 0.07],
  [184.81, 185.51, 0.0, 184.47, 0.07],
  [184.39, 184.46, 0.0, 184.07, 0.07],
  [183.96, 182.71, 0.0, 183.66, 0.07],
  [183.54, 182.84, 0.0, 183.23, 0.07],
  [183.12, 180.95, 0.0, 182.82, 0.07],
  [182.69, 182.35, 0.0, 182.4, 0.07],
  [182.27, 182.48, 0.0, 181.99, 0.07],
  [181.85, 180.38, 0.0, 181.57, 0.07],
  [181.42, 181.85, 0.0, 181.16, 0.07],
  [181.0, 180.43, 0.0, 180.74, 0.07],
  [180.57, 181.06, 0.0, 180.33, 0.07],
  [180.15, 180.08, 0.0, 179.9, 0.07],
  [179.72, 179.8, 0.0, 179.48, 0.07],
  [179.3, 177.27, 0.0, 179.05, 0.07],
  [178.88, 177.87, 0.0, 178.65, 0.07],
  [178.45, 178.12, 0.0, 178.25, 0.07],
  [178.03, 177.75, 0.0, 177.83, 0.07],
  [177.6, 176.76, 0.0, 177.41, 0.07],
  [177.18, 176.86, 0.0, 176.98, 0.07],
  [176.75, 177.38, 0.0, 176.58, 0.07],
  [176.33, 177.5, 0.0, 176.16, 0.07],
  [175.9, 176.44, 0.0, 175.75, 0.07],
  [175.48, 175.9, 0.0, 175.33, 0.07],
  [175.06, 174.5, 0.0, 174.94, 0.07],
  [174.63, 173.36, 0.0, 174.54, 0.07],
  [174.21, 173.04, 0.0, 174.13, 0.07],
  [173.78, 171.85, 0.0, 173.71, 0.07],
  [173.36, 173.26, 0.0, 173.28, 0.07],
  [172.93, 172.45, 0.0, 172.85, 0.07],
  [172.51, 172.79, 0.0, 172.43, 0.07],
  [172.08, 172.37, 0.0, 172.01, 0.07],
  [171.66, 173.54, 0.0, 171.6, 0.07],
  [171.23, 172.56, 0.0, 171.19, 0.07],
  [170.81, 170.1, 0.0, 170.76, 0.07],
  [170.39, 171.94, 0.0, 170.36, 0.07],
  [169.96, 169.16, 0.0, 169.91, 0.07],
  [169.54, 169.6, 0.0, 169.49, 0.07],
  [169.11, 168.19, 0.0, 169.08, 0.07],
  [168.69, 168.76, 0.0, 168.66, 0.07],
  [168.26, 165.92, 0.0, 168.22, 0.07],
  [167.84, 167.94, 0.0, 167.82, 0.07],
  [167.41, 166.88, 0.0, 167.4, 0.07],
  [166.99, 164.47, 0.0, 166.96, 0.07],
  [166.56, 168.08, 0.0, 166.53, 0.07],
  [166.14, 166.29, 0.0, 166.1, 0.07],
  [165.71, 164.84, 0.0, 165.67, 0.07],
  [165.29, 163.98, 0.0, 165.25, 0.07],
  [164.86, 163.2, 0.0, 164.83, 0.07],
  [164.44, 164.78, 0.0, 164.41, 0.07],
  [164.02, 164.75, 0.0, 163.98, 0.07],
  [163.59, 162.56, 0.0, 163.57, 0.07],
  [163.17, 164.09, 0.0, 163.16, 0.07],
  [162.74, 160.9, 0.0, 162.74, 0.07],
  [162.32, 162.97, 0.0, 162.34, 0.07],
  [161.89, 161.83, 0.0, 161.92, 0.07],
  [161.47, 161.45, 0.0, 161.5, 0.07],
  [161.04, 160.48, 0.0, 161.09, 0.07],
  [160.62, 160.63, 0.0, 160.67, 0.07],
  [160.19, 160.84, 0.0, 160.27, 0.07],
  [159.77, 158.46, 0.0, 159.89, 0.07],
  [159.34, 162.29, 0.0, 159.48, 0.07],
  [158.92, 159.13, 0.0, 159.06, 0.07],
  [158.49, 159.1, 0.0, 158.62, 0.07],
  [158.07, 157.69, 0.0, 158.19, 0.07],
  [157.65, 156.28, 0.0, 157.77, 0.07],
  [157.22, 158.08, 0.0, 157.36, 0.07],
  [156.8, 157.11, 0.0, 156.93, 0.07],
  [156.37, 155.37, 0.0, 156.51, 0.07],
  [155.95, 156.77, 0.0, 156.08, 0.07],
  [155.52, 156.16, 0.0, 155.62, 0.07],
  [155.1, 154.17, 0.0, 155.18, 0.07],
  [154.67, 155.31, 0.0, 154.75, 0.07],
  [154.25, 153.47, 0.0, 154.3, 0.07],
  [153.82, 152.39, 0.0, 153.87, 0.07],
  [153.4, 153.69, 0.0, 153.45, 0.07],
  [152.97, 152.7, 0.0, 153.06, 0.07],
  [152.55, 152.06, 0.0, 152.65, 0.07],
  [152.12, 153.54, 0.0, 152.22, 0.07],
  [151.7, 152.88, 0.0, 151.81, 0.07],
  [151.28, 150.26, 0.0, 151.39, 0.07],
  [150.85, 151.12, 0.0, 150.97, 0.07],
  [150.43, 151.73, 0.0, 150.55, 0.07],
  [150.0, 150.25, 0.0, 150.11, 0.07],
  [149.58, 150.17, 0.0, 149.69, 0.07],
  [149.15, 149.21, 0.0, 149.25, 0.07],
  [148.73, 150.42, 0.0, 148.87, 0.07],
  [148.3, 148.27, 0.0, 148.45, 0.07],
  [147.88, 147.86, 0.0, 148.03, 0.07],
  [147.45, 147.33, 0.0, 147.61, 0.07],
  [147.03, 146.81, 0.0, 147.18, 0.07],
  [146.6, 146.65, 0.0, 146.75, 0.07],
  [146.18, 148.03, 0.0, 146.32, 0.07],
  [145.75, 143.84, 0.0, 145.9, 0.07],
  [145.33, 143.54, 0.0, 145.49, 0.07],
  [144.9, 145.07, 0.0, 145.07, 0.07],
  [144.48, 144.07, 0.0, 144.66, 0.07],
  [144.06, 143.35, 0.0, 144.25, 0.07],
  [143.63, 143.55, 0.0, 143.81, 0.07],
  [143.21, 143.08, 0.0, 143.4, 0.07],
  [142.78, 142.61, 0.0, 142.98, 0.07],
  [142.36, 142.82, 0.0, 142.54, 0.07],
  [141.93, 143.27, 0.0, 142.12, 0.07],
  [141.51, 141.66, 0.0, 141.68, 0.07],
  [141.08, 139.89, 0.0, 141.25, 0.07],
  [140.66, 140.29, 0.0, 140.82, 0.07],
  [140.23, 139.57, 0.0, 140.39, 0.07],
  [139.81, 138.86, 0.0, 139.96, 0.07],
  [139.39, 140.82, 0.0, 139.54, 0.07],
  [138.96, 139.66, 0.0, 139.12, 0.07],
  [138.54, 137.02, 0.0, 138.69, 0.07],
  [138.11, 139.29, 0.0, 138.3, 0.07],
  [137.69, 138.3, 0.0, 137.9, 0.07],
  [137.26, 135.71, 0.0, 137.47, 0.07],
  [136.84, 135.65, 0.0, 137.05, 0.07],
  [136.42, 134.69, 0.0, 136.61, 0.07],
  [135.99, 135.48, 0.0, 136.17, 0.07],
  [135.57, 136.1, 0.0, 135.75, 0.07],
  [135.14, 135.34, 0.0, 135.32, 0.07],
  [134.72, 134.95, 0.0, 134.91, 0.07],
  [134.29, 133.76, 0.0, 134.47, 0.07],
  [133.87, 134.59, 0.0, 134.04, 0.07],
  [133.45, 133.11, 0.0, 133.63, 0.07],
  [133.02, 135.74, 0.0, 133.23, 0.07],
  [132.6, 133.68, 0.0, 132.8, 0.07],
  [132.17, 133.46, 0.0, 132.38, 0.07],
  [131.75, 132.42, 0.0, 131.95, 0.07],
  [131.33, 131.83, 0.0, 131.54, 0.07],
  [130.9, 132.0, 0.0, 131.11, 0.07],
  [130.48, 131.61, 0.0, 130.67, 0.07],
  [130.05, 130.57, 0.0, 130.26, 0.07],
  [129.63, 131.36, 0.0, 129.84, 0.07],
  [129.2, 128.01, 0.0, 129.4, 0.07],
  [128.78, 128.83, 0.0, 128.99, 0.07],
  [128.36, 127.29, 0.0, 128.57, 0.07],
  [127.93, 127.6, 0.0, 128.14, 0.07],
  [127.51, 126.54, 0.0, 127.71, 0.07],
  [127.08, 128.12, 0.0, 127.29, 0.07],
  [126.66, 127.09, 0.0, 126.87, 0.07],
  [126.23, 126.38, 0.0, 126.46, 0.07],
  [125.81, 125.45, 0.0, 126.06, 0.07],
  [125.39, 124.13, 0.0, 125.62, 0.07],
  [124.96, 125.33, 0.0, 125.19, 0.07],
  [124.54, 125.69, 0.0, 124.75, 0.07],
  [124.11, 125.79, 0.0, 124.33, 0.07],
  [123.69, 121.62, 0.0, 123.91, 0.07],
  [123.26, 123.34, 0.0, 123.48, 0.07],
  [122.84, 123.08, 0.0, 123.05, 0.07],
  [122.42, 124.78, 0.0, 122.62, 0.07],
  [121.99, 123.03, 0.0, 122.21, 0.07],
  [121.57, 121.51, 0.0, 121.79, 0.07],
  [121.14, 122.66, 0.0, 121.38, 0.07],
  [120.72, 122.04, 0.0, 120.95, 0.07],
  [120.29, 119.86, 0.0, 120.51, 0.07],
  [119.87, 120.23, 0.0, 120.09, 0.07],
  [119.45, 118.93, 0.0, 119.67, 0.07],
  [119.02, 119.18, 0.0, 119.22, 0.07],
  [118.6, 119.11, 0.0, 118.79, 0.07],
  [118.17, 117.57, 0.0, 118.36, 0.07],
  [117.75, 117.81, 0.0, 117.94, 0.07],
  [117.32, 118.23, 0.0, 117.53, 0.07],
  [116.9, 116.81, 0.0, 117.09, 0.07],
  [116.48, 115.9, 0.0, 116.66, 0.07],
  [116.05, 116.39, 0.0, 116.24, 0.07],
  [115.63, 116.15, 0.0, 115.82, 0.07],
  [115.2, 117.08, 0.0, 115.4, 0.07],
  [114.78, 115.93, 0.0, 115.0, 0.07],
  [114.35, 114.0, 0.0, 114.57, 0.07],
  [113.93, 113.43, 0.0, 114.13, 0.07],
  [113.51, 114.34, 0.0, 113.69, 0.07],
  [113.08, 111.2, 0.0, 113.24, 0.07],
  [112.66, 113.15, 0.0, 112.81, 0.07],
  [112.23, 110.62, 0.0, 112.37, 0.07],
  [111.81, 111.7, 0.0, 111.94, 0.07],
  [111.38, 109.68, 0.0, 111.51, 0.07],
  [110.96, 109.43, 0.0, 111.1, 0.07],
  [110.54, 110.34, 0.0, 110.65, 0.07],
  [110.11, 108.83, 0.0, 110.23, 0.07],
  [109.69, 109.26, 0.0, 109.81, 0.07],
  [109.26, 110.7, 0.0, 109.36, 0.07],
  [108.84, 108.73, 0.0, 108.96, 0.07],
  [108.41, 108.64, 0.0, 108.53, 0.07],
  [107.99, 109.4, 0.0, 108.11, 0.07],
  [107.57, 108.13, 0.0, 107.7, 0.07],
  [107.14, 106.59, 0.0, 107.27, 0.07],
  [106.72, 106.87, 0.0, 106.83, 0.07],
  [106.29, 107.92, 0.0, 106.41, 0.07],
  [105.87, 105.77, 0.0, 106.0, 0.07],
  [105.44, 105.18, 0.0, 105.58, 0.07],
  [105.02, 104.24, 0.0, 105.14, 0.07],
  [104.6, 103.13, 0.0, 104.71, 0.07],
  [104.17, 103.87, 0.0, 104.3, 0.07],
  [103.75, 103.74, 0.0, 103.86, 0.07],
  [103.32, 102.67, 0.0, 103.43, 0.07],
  [102.9, 102.27, 0.0, 103.0, 0.07],
  [102.48, 103.26, 0.0, 102.59, 0.07],
  [102.05, 102.85, 0.0, 102.17, 0.07],
  [101.63, 101.76, 0.0, 101.76, 0.07],
  [101.2, 99.59, 0.0, 101.34, 0.07],
  [100.78, 101.58, 0.0, 100.9, 0.07],
  [100.35, 100.87, 0.0, 100.48, 0.07],
  [99.93, 100.39, 0.0, 100.05, 0.07],
  [99.51, 99.58, 0.0, 99.61, 0.07],
  [99.08, 99.71, 0.0, 99.18, 0.07],
  [98.66, 98.03, 0.0, 98.72, 0.07],
  [98.23, 99.18, 0.0, 98.3, 0.07],
  [97.81, 97.67, 0.0, 97.86, 0.07],
  [97.38, 96.74, 0.0, 97.44, 0.07],
  [96.96, 97.97, 0.0, 97.02, 0.07],
  [96.54, 94.57, 0.0, 96.6, 0.07],
  [96.11, 97.24, 0.0, 96.21, 0.07],
  [95.69, 96.45, 0.0, 95.78, 0.07],
  [95.26, 96.71, 0.0, 95.38, 0.07],
  [94.84, 93.94, 0.0, 94.95, 0.07],
  [94.41, 92.64, 0.0, 94.53, 0.07],
  [93.99, 94.55, 0.0, 94.1, 0.07],
  [93.57, 93.88, 0.0, 93.67, 0.07],
  [93.14, 93.45, 0.0, 93.24, 0.07],
  [92.72, 91.99, 0.0, 92.8, 0.07],
  [92.29, 92.46, 0.0, 92.38, 0.07],
  [91.87, 91.71, 0.0, 91.99, 0.07],
  [91.44, 91.87, 0.0, 91.57, 0.07],
  [91.02, 90.67, 0.0, 91.14, 0.07],
  [90.6, 91.61, 0.0, 90.72, 0.07],
  [90.17, 88.13, 0.0, 90.27, 0.07],
  [89.75, 89.6, 0.0, 89.84, 0.07],
  [89.32, 88.87, 0.0, 89.41, 0.07],
  [88.9, 90.24, 0.0, 88.99, 0.07],
  [88.47, 87.15, 0.0, 88.56, 0.07],
  [88.05, 88.58, 0.0, 88.12, 0.07],
  [87.63, 88.94, 0.0, 87.72, 0.07],
  [87.2, 86.97, 0.0, 87.29, 0.07],
  [86.78, 86.07, 0.0, 86.89, 0.07],
  [86.35, 86.57, 0.0, 86.48, 0.07],
  [85.93, 85.15, 0.0, 86.05, 0.07],
  [85.5, 86.47, 0.0, 85.63, 0.07],
  [85.08, 84.16, 0.0, 85.2, 0.07],
  [84.66, 84.22, 0.0, 84.79, 0.07],
  [84.23, 82.34, 0.0, 84.36, 0.07],
  [83.81, 84.62, 0.0, 83.91, 0.07],
  [83.38, 84.71, 0.0, 83.49, 0.07],
  [82.96, 84.21, 0.0, 83.06, 0.07],
  [82.53, 82.41, 0.0, 82.63, 0.07],
  [82.11, 80.95, 0.0, 82.2, 0.07],
  [81.69, 81.06, 0.0, 81.76, 0.07],
  [81.26, 82.26, 0.0, 81.33, 0.07],
  [80.84, 79.95, 0.0, 80.91, 0.07],
  [80.41, 80.17, 0.0, 80.47, 0.07],
  [79.99, 79.22, 0.0, 80.07, 0.07],
  [79.56, 79.39, 0.0, 79.65, 0.07],
  [79.14, 78.72, 0.0, 79.22, 0.07],
  [78.72, 77.93, 0.0, 78.8, 0.07],
  [78.29, 78.72, 0.0, 78.39, 0.07],
  [77.87, 78.94, 0.0, 77.96, 0.07],
  [77.44, 78.34, 0.0, 77.56, 0.07],
  [77.02, 75.69, 0.0, 77.14, 0.07],
  [76.6, 75.78, 0.0, 76.7, 0.07],
  [76.17, 76.96, 0.0, 76.28, 0.07],
  [75.75, 77.43, 0.0, 75.87, 0.07],
  [75.32, 72.97, 0.0, 75.45, 0.07],
  [74.9, 74.85, 0.0, 75.03, 0.07],
  [74.47, 74.1, 0.0, 74.61, 0.07],
  [74.05, 75.75, 0.0, 74.19, 0.07],
  [73.63, 74.55, 0.0, 73.77, 0.07],
  [73.2, 73.74, 0.0, 73.36, 0.07],
  [72.78, 72.53, 0.0, 72.96, 0.07],
  [72.35, 72.46, 0.0, 72.55, 0.07],
  [71.93, 73.0, 0.0, 72.13, 0.07],
  [71.5, 70.75, 0.0, 71.71, 0.07],
  [71.08, 71.69, 0.0, 71.28, 0.07],
  [70.66, 70.73, 0.0, 70.88, 0.07],
  [70.23, 71.85, 0.0, 70.45, 0.07],
  [69.81, 69.76, 0.0, 70.03, 0.07],
  [69.38, 69.7, 0.0, 69.61, 0.07],
  [68.96, 68.0, 0.0, 69.17, 0.07],
  [68.53, 70.13, 0.0, 68.75, 0.07],
  [68.11, 67.64, 0.0, 68.32, 0.07],
  [67.69, 67.26, 0.0, 67.89, 0.07],
  [67.26, 67.97, 0.0, 67.49, 0.07],
  [66.84, 66.94, 0.0, 67.07, 0.07],
  [66.41, 66.82, 0.0, 66.63, 0.07],
  [65.99, 65.67, 0.0, 66.21, 0.07],
  [65.56, 65.75, 0.0, 65.82, 0.07],
  [65.14, 64.38, 0.0, 65.4, 0.07],
  [64.72, 65.29, 0.0, 64.96, 0.07],
  [64.29, 66.52, 0.0, 64.54, 0.07],
  [63.87, 64.45, 0.0, 64.12, 0.07],
  [63.44, 63.12, 0.0, 63.68, 0.07],
  [63.02, 64.25, 0.0, 63.27, 0.07],
  [62.59, 63.08, 0.0, 62.83, 0.07],
  [62.17, 63.44, 0.0, 62.41, 0.07],
  [61.75, 61.78, 0.0, 61.98, 0.07],
  [61.32, 62.07, 0.0, 61.55, 0.07],
  [60.9, 60.35, 0.0, 61.11, 0.07],
  [60.47, 60.36, 0.0, 60.68, 0.07],
  [60.05, 59.38, 0.0, 60.23, 0.07],
  [59.62, 61.11, 0.0, 59.8, 0.07],
  [59.2, 58.97, 0.0, 59.37, 0.07],
  [58.78, 58.24, 0.0, 58.96, 0.07],
  [58.35, 57.69, 0.0, 58.52, 0.07],
  [57.93, 59.12, 0.0, 58.11, 0.07],
  [57.5, 58.61, 0.0, 57.69, 0.07],
  [57.08, 58.05, 0.0, 57.29, 0.07],
  [56.65, 55.22, 0.0, 56.84, 0.07],
  [56.23, 55.32, 0.0, 56.39, 0.07],
  [55.81, 56.72, 0.0, 55.98, 0.07],
  [55.38, 56.2, 0.0, 55.55, 0.07],
  [54.96, 55.94, 0.0, 55.1, 0.07],
  [54.53, 53.27, 0.0, 54.68, 0.07],
  [54.11, 54.4, 0.0, 54.23, 0.07],
  [53.68, 54.91, 0.0, 53.82, 0.07],
  [53.26, 52.95, 0.0, 53.43, 0.07],
  [52.84, 53.03, 0.0, 52.99, 0.07],
  [52.41, 51.19, 0.0, 52.58, 0.07],
  [51.99, 52.25, 0.0, 52.16, 0.07],
  [51.56, 51.21, 0.0, 51.75, 0.07],
  [51.14, 51.03, 0.0, 51.3, 0.07],
  [50.71, 50.23, 0.0, 50.86, 0.07],
  [50.29, 50.88, 0.0, 50.42, 0.07],
  [49.87, 48.11, 0.0, 50.0, 0.07],
  [49.44, 49.07, 0.0, 49.58, 0.07],
  [49.02, 49.37, 0.0, 49.17, 0.07],
  [48.59, 48.25, 0.0, 48.75, 0.07],
  [48.17, 48.72, 0.0, 48.33, 0.07],
  [47.75, 46.53, 0.0, 47.9, 0.07],
  [47.32, 47.86, 0.0, 47.46, 0.07],
  [46.9, 46.33, 0.0, 47.02, 0.07],
  [46.47, 46.4, 0.0, 46.59, 0.07],
  [46.05, 47.16, 0.0, 46.18, 0.07],
  [45.62, 44.49, 0.0, 45.75, 0.07],
  [45.2, 44.45, 0.0, 45.31, 0.07],
  [44.78, 43.6, 0.0, 44.89, 0.07],
  [44.35, 45.69, 0.0, 44.47, 0.07],
  [43.93, 43.85, 0.0, 44.05, 0.07],
  [43.5, 44.43, 0.0, 43.64, 0.07],
  [43.08, 45.37, 0.0, 43.23, 0.07],
  [42.65, 42.73, 0.0, 42.81, 0.07],
  [42.23, 40.46, 0.0, 42.35, 0.07],
  [41.81, 41.67, 0.0, 41.93, 0.07],
  [41.38, 42.52, 0.0, 41.5, 0.07],
  [40.96, 42.23, 0.0, 41.08, 0.07],
  [40.53, 38.46, 0.0, 40.66, 0.07],
  [40.11, 40.58, 0.0, 40.24, 0.07],
  [39.68, 41.02, 0.0, 39.8, 0.07],
  [39.26, 39.85, 0.0, 39.41, 0.07],
  [38.84, 40.73, 0.0, 38.98, 0.07],
  [38.41, 38.95, 0.0, 38.56, 0.07],
  [37.99, 40.76, 0.0, 38.14, 0.07],
  [37.56, 36.53, 0.0, 37.72, 0.07],
  [37.14, 36.88, 0.0, 37.28, 0.07],
  [36.71, 35.93, 0.0, 36.84, 0.07],
  [36.29, 34.98, 0.0, 36.4, 0.07],
  [35.87, 37.11, 0.0, 35.98, 0.07],
  [35.44, 35.64, 0.0, 35.58, 0.07],
  [35.02, 34.29, 0.0, 35.15, 0.07],
  [34.59, 33.83, 0.0, 34.72, 0.07],
  [34.17, 35.83, 0.0, 34.31, 0.07],
  [33.74, 33.99, 0.0, 33.91, 0.07],
  [33.32, 32.34, 0.0, 33.48, 0.07],
  [32.9, 34.28, 0.0, 33.05, 0.07],
  [32.47, 32.54, 0.0, 32.63, 0.07],
  [32.05, 32.83, 0.0, 32.2, 0.07],
  [31.62, 31.94, 0.0, 31.75, 0.07],
  [31.2, 29.93, 0.0, 31.31, 0.07],
  [30.77, 31.26, 0.0, 30.91, 0.07],
  [30.35, 29.76, 0.0, 30.47, 0.07],
  [29.93, 31.24, 0.0, 30.06, 0.07],
  [29.5, 30.29, 0.0, 29.65, 0.07],
  [29.08, 28.6, 0.0, 29.21, 0.07],
  [28.65, 29.28, 0.0, 28.78, 0.07],
  [28.23, 27.03, 0.0, 28.36, 0.07],
  [27.8, 27.61, 0.0, 27.96, 0.07],
  [27.38, 25.86, 0.0, 27.53, 0.07],
  [26.96, 26.33, 0.0, 27.11, 0.07],
  [26.53, 25.84, 0.0, 26.67, 0.07],
  [26.11, 26.67, 0.0, 26.25, 0.07],
  [25.68, 25.02, 0.0, 25.83, 0.07],
  [25.26, 24.11, 0.0, 25.38, 0.07],
  [24.83, 25.13, 0.0, 24.96, 0.07],
  [24.41, 24.22, 0.0, 24.55, 0.07],
  [23.99, 25.09, 0.0, 24.13, 0.07],
  [23.56, 23.67, 0.0, 23.69, 0.07],
  [23.14, 23.91, 0.0, 23.28, 0.07],
  [22.71, 22.23, 0.0, 22.87, 0.07],
  [22.29, 22.17, 0.0, 22.44, 0.07],
  [21.86, 24.23, 0.0, 22.04, 0.07],
  [21.44, 21.55, 0.0, 21.6, 0.07],
  [21.02, 20.96, 0.0, 21.19, 0.07],
  [20.59, 21.42, 0.0, 20.76, 0.07],
  [20.17, 21.07, 0.0, 20.37, 0.07],
  [19.74, 19.8, 0.0, 19.95, 0.07],
  [19.32, 20.82, 0.0, 19.55, 0.07],
  [18.9, 17.28, 0.0, 19.13, 0.07],
  [18.47, 18.25, 0.0, 18.7, 0.07],
  [18.05, 18.62, 0.0, 18.27, 0.07],
  [17.62, 17.21, 0.0, 17.86, 0.07],
  [17.2, 17.62, 0.0, 17.43, 0.07],
  [16.77, 15.01, 0.0, 17.0, 0.07],
  [16.35, 15.43, 0.0, 16.56, 0.07],
  [15.93, 16.71, 0.0, 16.15, 0.07],
  [15.5, 14.37, 0.0, 15.76, 0.07],
  [15.08, 15.91, 0.0, 15.34, 0.07],
  [14.65, 15.32, 0.0, 14.94, 0.07],
  [14.23, 13.24, 0.0, 14.51, 0.07],
  [13.8, 12.73, 0.0, 14.07, 0.07],
  [13.38, 12.28, 0.0, 13.64, 0.07],
  [12.96, 11.24, 0.0, 13.19, 0.07],
  [12.53, 12.3, 0.0, 12.78, 0.07],
  [12.11, 11.91, 0.0, 12.37, 0.07],
  [11.68, 11.14, 0.0, 11.94, 0.07],
  [11.26, 11.05, 0.0, 11.52, 0.07],
  [10.83, 9.29, 0.0, 11.09, 0.07],
  [10.41, 9.94, 0.0, 10.66, 0.07],
  [9.99, 10.08, 0.0, 10.25, 0.07],
  [9.56, 8.94, 0.0, 9.83, 0.07],
  [9.14, 11.83, 0.0, 9.43, 0.07],
  [8.71, 8.18, 0.0, 9.0, 0.07],
  [8.29, 7.31, 0.0, 8.56, 0.07],
  [7.86, 8.04, 0.0, 8.14, 0.07],
  [7.44, 9.2, 0.0, 7.72, 0.07],
  [7.02, 6.6, 0.0, 7.28, 0.07],
  [6.59, 5.59, 0.0, 6.86, 0.07],
  [6.17, 7.08, 0.0, 6.44, 0.07],
  [5.74, 5.64, 0.0, 6.0, 0.07],
  [5.32, 2.74, 0.0, 5.56, 0.07],
  [4.89, 5.72, 0.0, 5.14, 0.07],
  [4.47, 5.64, 0.0, 4.71, 0.07],
  [4.05, 3.56, 0.0, 4.28, 0.07],
  [3.62, 3.92, 0.0, 3.85, 0.07],
  [3.2, 3.44, 0.0, 3.4, 0.07],
  [2.77, 3.76, 0.0, 2.98, 0.07],
  [2.35, 3.51, 0.0, 2.56, 0.07],
  [1.92, 2.29, 0.0, 2.16, 0.07],
  [1.5, -0.6, 0.0, 1.73, 0.07],
  [1.08, -0.06, 0.0, 1.3, 0.07],
  [0.65, 2.66, 0.0, 0.88, 0.07],
  [0.23, -0.58, 0.0, 0.48, 0.07],
];
