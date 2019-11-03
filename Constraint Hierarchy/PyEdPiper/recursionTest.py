# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 12:10:29 2019

@author: zarif
"""

d = {}

def go(start,end):
    if start==end:
        print("--> ",start)
        return
    mid = int((start+end)/2)
    
    go(mid+1,end)
    d[(start,end)] = mid
    print(start," , ",mid," , ",end)
    go(start,mid)
    
def go2(start,end):
    el = (start,end)
    if (el in d)==False:
        return
    mid = int((start+end)/2)
    go2(start,mid)
    print(start," -- ",mid," -- ",end)
    go2(mid,end)
    
    
if __name__ == '__main__':
    #go(1,10)
    #print(d)
    #go2(1,10)
    li=[]
    li.append((1,5))
    li.append((3,6))
    li.append((2,5))
    
    for el in li:
        print(el[0])
    
    li.remove((2,5))
