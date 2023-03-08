function [lat, lon, alt, t] = BSP_mapCoordsFcn(fds)

group = 'GPS';

t = kVIS_fdsGetChannel(fds, group, 'Time');

lon = kVIS_fdsGetChannel(fds, group, 'lon');

lat = kVIS_fdsGetChannel(fds, group, 'lat');

alt = kVIS_fdsGetChannel(fds, group, 'alt');
alt(alt < 0) = 0;
alt = alt - min(alt);

end