function result = lonLat2Mercator(lon,lat)
x=lon*20037508.34/180;
y=log(tan((90+lat)*pi/360))/(pi/180);
y=y*20037508.34/180;
result.X=x;
result.Y=y;
return