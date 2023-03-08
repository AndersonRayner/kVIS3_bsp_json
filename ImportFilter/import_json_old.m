% kVIS3 Data Visualisation
%
% Copyright (C) 2012 - present  Kai Lehmkuehler, Matt Anderson and
% contributors
%
% Contact: kvis3@uav-flightresearch.com
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

function [] = import_json(hObject, ~)


if ~nargin
    % Debugging
    file = 'C:\Users\matt\Downloads\jumpAeroData\pitch-stand-tests_10_2023-03-06-10-33-55.json'

else
    % Select PX4 Log file
    [fileName, pathName] = uigetfile('*.json');
    file = fullfile(pathName,fileName);
end

time_data = 't';

tic

fprintf('Importing JSON file\n');
fprintf('\t%s\n\n',file);

%% get new fds structure
fds = kVIS_fdsInitNew();

fds.BoardSupportPackage = 'json';

[fds, parentNode] = kVIS_fdsAddTreeBranch(fds, 0, 'json_data');

%% Import json file into struct to read
fid = fopen(file, 'r');
if fid == -1
  error('Cannot open file: %s', file);
end

rawText = fscanf(fid, '%s', inf);
fclose(fid);

% Decode json file
dataRaw = jsondecode(rawText);

% Clean up the data
groups = fieldnames(dataRaw);

for ii = 1:numel(groups)
    field = groups{ii};
    subfields = fieldnames(dataRaw.(field));
    json.(field) = dataRaw.(field).(subfields{1});
end

% Find the time vector
t = json.(time_data); ...
    t = t - t(1);

%% Create kVIS elements
groups = fieldnames(json);

% Loop through each group name
for ii = 1:numel(groups)
    
    % Extract log name
    groupName = groups{ii};
    fprintf('\tImporting group %s\n',groupName);

    if (numel(size(json.(groupName))) ~= 2)
        fprintf('\t\tSkipping %s as not size 2\n',groupName);
        continue
    end

    % Skip the 'time' entry
    if (groupName == time_data)
        continue
    end

    % Get the data, put time vector at start
    data = [t,json.(groupName)];

    % Create the variable names/units/frames
    n_samples = size(data,1);
    n_channels = size(data,2);
    varNames = cell(n_channels,1); ...
        varUnits = varNames; ...
        varFrames = varNames;
    for jj = 1:n_channels
        varNames{jj} = sprintf('Var_%d',jj-1);
        varUnits{jj} = 'N/A';
        varFrames{jj} = 'Unknown Frame';
    end
   
    % Fix the time element
    varNames{1} = 'Time';
    varUnits{1} = 's';
    varFrames{1} = '';
%     data(:,1) = data(:,1) / 1e6;
    
    % Find t_start and t_end
%     t_start = min(t_start,data(1,1));
%     t_end   = max(t_end  ,data(end,1));

    % Add euler fields to quaternions entry ([t, qw, qx, qy, qz])
    if contains(groupName,'attitude') && (n_channels == 5)
        % Add extra stuff to the labelling
        varNames  = [varNames; {'roll'};{'pitch'};{'yaw' }];
        varUnits  = [varUnits; {'deg' };{'deg'  };{'deg' }];
        varFrames = [varFrames;{'body'};{'body' };{'body'}];
        
        % Find which variables to use
        qw = data(:,2);
        qx = data(:,3);
        qy = data(:,4);
        qz = data(:,5);
        
        % Convert quaternions to euler angles and store
        quat_angles = [qw, qx, qy, qz];
        euler_angles = q2e(quat_angles)*180.0/pi;
        data = [ data, euler_angles ];
    end
      
    % Generate the kVIS data structure
    fds = kVIS_fdsAddTreeLeaf(fds, groupName, varNames, varNames, varUnits, varFrames, data, parentNode, false);
 
end

%% Update kVIS
fds = kVIS_fdsUpdateAttributes(fds);

[~,name,~] = fileparts(file);
kVIS_addDataSet(hObject, fds, matlab.lang.makeValidName(name));

fprintf('File imported in %.2f seconds\n',toc);

return

end

function euler_angles = q2e(q)
%
%
% Q2E converts Quaternions to roll-pitch-yaw (1-2-3) sequence Euler angles
%
% References:
% + Dan Newman: C130 Kalman Filter - checked
% + Diebel2006: Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors.pdf - checked
%
% Output angles in degrees

q0 = q(:,1);
q1 = q(:,2);
q2 = q(:,3);
q3 = q(:,4);

% convert quaternions to euler angles 1-2-3
phi   = atan2( 2*(q2.*q3 + q0.*q1), q0.^2 - q1.^2 - q2.^2 + q3.^2);
theta = asin(2*(q0.*q2 - q1.*q3));
psi   = atan2( 2*(q1.*q2 + q0.*q3), q0.^2 + q1.^2 - q2.^2 - q3.^2);

euler_angles = [phi,theta,psi];

return
end