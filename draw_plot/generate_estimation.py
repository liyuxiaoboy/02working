# -*-coding:utf-8 -*-
#!/usr/bin/python3
import sys
import codecs
import re
import decimal

if __name__ == '__main__':
    file_name = sys.argv[1]
    timestamps = sys.argv[2]
    out_name = sys.argv[3]
    reader = codecs.open(file_name,'r',encoding='utf-8')
    timer = codecs.open(timestamps,'r',encoding='utf-8')
    writer = codecs.open(out_name,'w',encoding='utf-8')

    line1=reader.readlines()
    line2=timer.readlines()

    linetime=line2[0]
    time_=linetime.strip().split()
    timebegin=time_[0]
    # print timebegin
    timer.close()

    data={}
    time_now=0.0
    time_pro=0.0
    # print timebegin
    for i in range(len(line1)):
        if i == 0:
            time_now=decimal.Decimal(timebegin)
            line_ref=line1[i]
            word_ref=line_ref.strip().split(",")
            time_pro=word_ref[0]
            data[time_now]=str(time_now)+' '+word_ref[1]+' '+word_ref[2]+' '+word_ref[3]+' '+word_ref[5]+' '+word_ref[6]+' '+word_ref[7]+' '+word_ref[4]+'\n'
        else :
            line_ref=line1[i] 
            word_ref=line_ref.strip().split(",")
            time_ref=word_ref[0]
            
            dist=(decimal.Decimal(time_ref)-decimal.Decimal(time_pro))/1000
            print dist
            time_now=time_now+dist
            time_pro=time_ref
            data[time_now]=str(time_now)+' '+word_ref[1]+' '+word_ref[2]+' '+word_ref[3]+' '+word_ref[5]+' '+word_ref[6]+' '+word_ref[7]+' '+word_ref[4]+'\n'
    
    sort_data=sorted(data.keys())
    for i in sort_data:
        writer.write(data[i])


    
