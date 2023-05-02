import csv


if __name__=="__main__":
  data = []
  # # opening the CSV file
  with open('/home/amkyu/slam_ws/src/FAST-LIVO/results/est_odom.csv', mode ='r')as file:
   
    # reading the CSV file
    csvFile = csv.reader(file)
  
    # displaying the contents of the CSV file
    for i, lines in enumerate(csvFile):
      if i == 0:
        data.append(lines)
      else:
        data.append(lines[0:3])
      # print(lines)
      
  
  f = open("/home/amkyu/slam_ws/src/FAST-LIVO/src/ceres_bin/poses_optimized.txt", "r")
  for i, x in enumerate(f):
    fields = x.split()
    data[i+1].extend(fields[1:])

  f.close()

  with open('/home/amkyu/slam_ws/src/FAST-LIVO/results/est_odom.csv', mode ='r')as file:
   
    # reading the CSV file
    csvFile = csv.reader(file)
  
    # displaying the contents of the CSV file
    for i, lines in enumerate(csvFile):
      if i != 0:
        data[i].extend(lines[10:])

  with open('/home/amkyu/slam_ws/src/FAST-LIVO/results/opt_est_odom.csv', "w") as csv_file:
    csvwriter = csv.writer(csv_file) 
    # writing the data rows 
    csvwriter.writerows(data)

    # print(fields)

  