reset
set tics
unset key
set title "The result of Corner Stitching"
set object 1 rect from 0,175000 to 29500,300000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 2 rect from 29500,175000 to 90000,300000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 3 rect from 90000,175000 to 100000,300000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 4 rect from 100000,200000 to 107000,300000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 5 rect from 107000,260000 to 200000,300000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 6 rect from 107000,200000 to 160500,260000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 7 rect from 100000,175000 to 160500,200000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 8 rect from 0,150000 to 160500,175000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 9 rect from 160500,150000 to 167500,260000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 10 rect from 167500,150000 to 170500,260000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 11 rect from 0,135000 to 170500,150000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 12 rect from 0,87500 to 23400,135000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 13 rect from 23400,87500 to 29500,135000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 14 rect from 29500,87500 to 39500,135000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 15 rect from 39500,110000 to 170500,135000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 16 rect from 39500,87500 to 100000,110000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 17 rect from 0,85000 to 100000,87500 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 18 rect from 0,20000 to 23400,85000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 19 rect from 23400,20000 to 39500,85000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 20 rect from 39500,20000 to 50000,85000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 21 rect from 0,0 to 50000,20000 fc rgb 'gold' fs transparent solid 0.4 border lc rgb '#010101'
set object 22 rect from 50000,0 to 100000,85000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 23 rect from 100000,0 to 110000,110000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 24 rect from 110000,0 to 170500,110000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 25 rect from 170500,0 to 177500,260000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 26 rect from 177500,231500 to 200000,260000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 27 rect from 200000,250000 to 400000,300000 fc rgb 'gold' fs transparent solid 0.4 border lc rgb '#010101'
set object 28 rect from 177500,0 to 200000,231500 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 29 rect from 200000,0 to 240000,250000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 30 rect from 240000,90000 to 275000,250000 fc rgb '#00ff00' fs transparent solid 0.4 border lc rgb '#010101'
set object 31 rect from 275000,90000 to 400000,250000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set object 32 rect from 240000,0 to 400000,90000 fc rgb '#0f0f0f' fs transparent solid 0.4 border lc rgb '#010101'
set style line 1 lc rgb "red" lw 3
set border ls 1
set terminal png
set output "corner_stitch.png"
plot [0:400000][0:300000] 'line' w l lt 2 lw 1
set terminal x11 persist
replot
exit