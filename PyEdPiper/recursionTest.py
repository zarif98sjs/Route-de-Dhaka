# -*- coding: utf-8 -*-
"""
Created on Thu Oct 31 12:10:29 2019

@author: zarif
"""

def go(start,end):
    if start==end:
        return
    mid = int((start+end)/2)
    
    go(mid+1,end)
    print(start," , ",mid," , ",end)
    go(start,mid)
    
if __name__ == '__main__':
    go(4,18)