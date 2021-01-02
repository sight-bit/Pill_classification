import csv
import io

# innit csv file  
f = io.open('Pill_list.csv', 'r', encoding='utf-8')
r = list(csv.reader(f))
