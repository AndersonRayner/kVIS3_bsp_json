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

function fds = import_json(file)


if ~nargin
    % Debugging
    file = 'C:\Users\matt\Downloads\jumpAeroData\matlab_stuff\data\pitch-stand-tests_106_2023-03-07-15-11-14.json';

end

time_data = 't';

tic

fprintf('Importing JSON file\n');
fprintf('\t%s\n\n',file);

%% get new fds structure
fds = kVIS_fdsInitNew();

fds.BoardSupportPackage = 'json';

[fds, rootNode] = kVIS_fdsAddTreeBranch(fds, 0, 'json_data');

%% Import json file into struct to read
fid = fopen(file, 'r');
if fid == -1
  error('Cannot open file: %s', file);
end

rawText = fscanf(fid, '%s', inf);
fclose(fid);

% Replace nans with 0 (as matlab doesn't like it)
rawText = strrep(rawText,'NaN','0.0');

if rawText(end) ~= ']'
    rawText(end+1) = ']';
end

if rawText(end-1) ~= '}'
    rawText(end) = '}';
    rawText(end+1) = ']';
end

% Decode json file
dataRaw = jsondecode(rawText);

% Each timestep is a it's own json thing, so yeah....
n_samples = numel(dataRaw);

dataStruct = struct();

for ii = 1:n_samples

%     fprintf('Index %d\n',ii);

    % Get the fields for this timestep
    fields = fieldnames(dataRaw{ii});
    
    % Skip if empty
    if isempty(fields)
        continue
    end

    % Check if we already have this field
    seenFields = fieldnames(dataStruct);
  
    if isempty(seenFields)
        seenFields = "";
    end

    % Loop through each of the fields for this timestep
    for jj = 1:numel(fields)
        field = fields{jj};

        % Extract data for this timestep
            if (isstruct(dataRaw{ii}.(field)))
                % Thing is a struct, so we'll have to recurse it
                temp = returnStructedData(dataRaw{ii}.(field));
            else
                temp = dataRaw{ii}.(field);
            end

        % Skip anthing matrix-sized or above
        if min(size(temp)) ~= 1
            continue;
        end

        % Create somewhere to put it (if it doesn't exist)
        if max(contains(seenFields,field)) == 0 
            % Create the field
            fprintf('\tFound field %s\n',field);
            dataStruct.(field) = nan(n_samples,numel(temp));

        end

        % Fill the data for this round
        dataStruct.(field)(ii,:) = temp;

    end

end

%% Create kVIS elements
t = dataStruct.t; ...
    t = t - min(t);

groups = fieldnames(dataStruct);

% Loop through each group name
for ii = 1:numel(groups)
    
    % Extract log name
    groupName = groups{ii};
    fprintf('\tImporting group %s\n',groupName);

    if (numel(size(dataStruct.(groupName))) ~= 2)
        fprintf('\t\tSkipping %s as not size 2\n',groupName);
        keyboard
        continue
    end

    % Skip the 'time' entry
    if (groupName == time_data)
        continue
    end

    % Get the data, put time vector at start
    data = [t,dataStruct.(groupName)];

    % Remove any elements that have a time of nan
    idx = isnan(data(:,1));
    data(idx,:) = [];

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
    if contains(groupName,'_')
            idx = find(groupName=='_');

            name_before = groupName(1:idx(1)-1);
            groupName = groupName(idx(1)+1:end);

            [~, parentNode] = kVIS_fdsGetGroup(fds, name_before);

            if parentNode <= 0
                [ fds, parentNode ] = kVIS_fdsAddTreeBranch(fds, 1, name_before);
            end
    else
        parentNode = rootNode;
    end
    fds = kVIS_fdsAddTreeLeaf(fds, groupName, varNames, varNames, varUnits, varFrames, data, parentNode, false);
 
end

%% Update kVIS
fds = kVIS_fdsUpdateAttributes(fds);

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


function data = returnStructedData(data)

    while isstruct(data)
        fields = fieldnames(data);
        data = data.(fields{1});
    end

    return;

end
