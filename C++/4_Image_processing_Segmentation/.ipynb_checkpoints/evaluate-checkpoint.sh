#!/bin/bash

meanIoU=0
count=0
for i in retinas/images/*.tif 
do
	filename=$(basename "$i")
	n=${filename%_*}

	./retinaSegment $i retinas/mask/${n}_test_mask.tif retinas/output/${n}_test_output.tif
	IoU="$(./iou retinas/output/${n}_test_output.tif retinas/1st_manual/${n}_manual1.tif)"

	echo $filename = $IoU
	meanIoU=`echo $meanIoU+$IoU|bc`
	let "count=count+1"
done

echo "-------"
echo -n "Media IoU = "
echo "scale=5; $meanIoU/$count"|bc -l
