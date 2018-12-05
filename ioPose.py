# -*-coding:utf-8 -*-
#!/usr/bin/python3
import sys
import codecs
import re

if __name__ == '__main__':
    file_name = sys.argv[1]
    out_name = sys.argv[2]
    reader = codecs.open(file_name,'r',encoding='utf-8')
    writer = codecs.open(out_name,'w',encoding='utf-8')
    # for line in reader:
    #     text = line.strip().split()
    #     word = text[1]
    #     writer.write(word+' '+'rgb/'+word+'.png')
    #     writer.write('\n')
    
    
    lines = reader.readlines()
    	
    for i in range(len(lines)):
	if i%38<12 :
    		line=lines[i]
        	if re.match('pcd_filename',line.strip()):
        		pcd_filename = line.strip().split('"')
        		word = pcd_filename[1]
        		writer.write(word+' ')
        	if re.match('timestamp',line.strip()):
        		timestamp = line.strip().split()
        		word = timestamp[1]
        		writer.write(word+' ')
        	if re.match('x:',line.strip()):
        		x_ = line.strip().split()
        		word = x_[1]
        		writer.write(word+' ')
        	if re.match('y:',line.strip()):
        		y_ = line.strip().split()
        		word = y_[1]
        		writer.write(word+' ')
        	if re.match('z:',line.strip()):
        		z_ = line.strip().split()
        		word = z_[1]
        		writer.write(word+' ')
        	if re.match('qw:',line.strip()):
        		qw= line.strip().split()
        		word = qw[1]
        		writer.write(word+' ')
		if re.match('qx:',line.strip()):
        		qx= line.strip().split()
        		word = qx[1]
        		writer.write(word+' ')
		if re.match('qy:',line.strip()):
        		qy = line.strip().split()
        		word = qy[1]
        		writer.write(word+' ')
		if re.match('qz:',line.strip()):
        		qz= line.strip().split()
        		word = qz[1]
        		writer.write(word+'\n')
		else:
        		continue

	else:
		continue
	
        
