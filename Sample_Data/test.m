

fileName = 'sampleData.json';

% Import the data
fid = fopen(fileName, 'r');
if fid == -1
  error('Cannot open file: %s', fileName);
end

rawText = fscanf(fid, '%s', inf);
fclose(fid);

% Decode json file
dataRaw = jsondecode(rawText);

% Clean up the data
fields = fieldnames(dataRaw);

for ii = 1:numel(fields)
    field = fields{ii};
    subfields = fieldnames(dataRaw.(field));
    data.(field) = dataRaw.(field).(subfields{1});
end


% Plots
figure(1); clf; 
