#!/usr/bin/python
import sys
import csv

def extract_data(files):
    data = []
    for data_file in files:
        with open(data_file, 'rb') as f:
            reader = csv.reader(f)
            for row in reader:
                data.append(row)
            print "Processing: %s , %s rows" %(data_file, reader.line_num)
    print "Final Length: ", len(data) 
    return data

if __name__=='__main__':
   files = sys.argv[1:]
   data = extract_data(files)
   with open('condensed_data.csv', 'wb') as f_out:
       writer = csv.writer(f_out)
       writer.writerows(data)
