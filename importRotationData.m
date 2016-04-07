%% Import data from text file.
% Script for importing data from the following text file:
%
%    E:\FS Testing - ST2 - Test 1 - 03-13-16.log
%
% To extend the code to different selected data or a different text file,
% generate a function instead of a script.

% Auto-generated by MATLAB on 2016/03/14 10:25:33

%% Initialize variables.
filename = 'C:\Users\clk0032\Dropbox\Friction Connection Research\Full Scale Test Data\FS Testing -ST2 - 04-07-16\FS Testing - ST2 - Test 2 - 04-07-16.log';
delimiter = ',';
startRow = 10;

%% Read columns of data as strings:
% For more information, see the TEXTSCAN documentation.
formatSpec = '%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%[^\n\r]';

%% Open the text file.
fileID = fopen(filename,'r');

%% Read columns of data according to format string.
% This call is based on the structure of the file used to generate this
% code. If an error occurs for a different file, try regenerating the code
% from the Import Tool.
textscan(fileID, '%[^\n\r]', startRow-1, 'ReturnOnError', false);
dataArray = textscan(fileID, formatSpec, 'Delimiter', delimiter, 'ReturnOnError', false);

%% Close the text file.
fclose(fileID);

%% Convert the contents of columns containing numeric strings to numbers.
% Replace non-numeric strings with NaN.
raw = repmat({''},length(dataArray{1}),length(dataArray)-1);
for col=1:length(dataArray)-1
    raw(1:length(dataArray{col}),col) = dataArray{col};
end
numericData = NaN(size(dataArray{1},1),size(dataArray,2));

for col=[1,3,4,5,6,8,9,10,11,13,14,15,16,18,19,20,21]
    % Converts strings in the input cell array to numbers. Replaced non-numeric
    % strings with NaN.
    rawData = dataArray{col};
    for row=1:size(rawData, 1);
        % Create a regular expression to detect and remove non-numeric prefixes and
        % suffixes.
        regexstr = '(?<prefix>.*?)(?<numbers>([-]*(\d+[\,]*)+[\.]{0,1}\d*[eEdD]{0,1}[-+]*\d*[i]{0,1})|([-]*(\d+[\,]*)*[\.]{1,1}\d+[eEdD]{0,1}[-+]*\d*[i]{0,1}))(?<suffix>.*)';
        try
            result = regexp(rawData{row}, regexstr, 'names');
            numbers = result.numbers;
            
            % Detected commas in non-thousand locations.
            invalidThousandsSeparator = false;
            if any(numbers==',');
                thousandsRegExp = '^\d+?(\,\d{3})*\.{0,1}\d*$';
                if isempty(regexp(thousandsRegExp, ',', 'once'));
                    numbers = NaN;
                    invalidThousandsSeparator = true;
                end
            end
            % Convert numeric strings to numbers.
            if ~invalidThousandsSeparator;
                numbers = textscan(strrep(numbers, ',', ''), '%f');
                numericData(row, col) = numbers{1};
                raw{row, col} = numbers{1};
            end
        catch me
        end
    end
end


%% Split data into numeric and cell columns.
rawNumericColumns = raw(:, [1,3,4,5,6,8,9,10,11,13,14,15,16,18,19,20,21]);
rawCellColumns = raw(:, [2,7,12,17]);


%% Replace non-numeric cells with NaN
R = cellfun(@(x) ~isnumeric(x) && ~islogical(x),rawNumericColumns); % Find non-numeric cells
rawNumericColumns(R) = {NaN}; % Replace non-numeric cells

%% Allocate imported array to column variable names
VarName1 = cell2mat(rawNumericColumns(:, 1));
VarName2 = rawCellColumns(:, 1);
CalA1 = cell2mat(rawNumericColumns(:, 2));
CalA2 = cell2mat(rawNumericColumns(:, 3));
CalA3 = cell2mat(rawNumericColumns(:, 4));
CalA4 = cell2mat(rawNumericColumns(:, 5));
VarName7 = rawCellColumns(:, 2);
CalB1 = cell2mat(rawNumericColumns(:, 6));
CalB2 = cell2mat(rawNumericColumns(:, 7));
CalB3 = cell2mat(rawNumericColumns(:, 8));
CalB4 = cell2mat(rawNumericColumns(:, 9));
VarName12 = rawCellColumns(:, 3);
Aw = cell2mat(rawNumericColumns(:, 10));
Ax = cell2mat(rawNumericColumns(:, 11));
Ay = cell2mat(rawNumericColumns(:, 12));
Az = cell2mat(rawNumericColumns(:, 13));
VarName17 = rawCellColumns(:, 4);
Bw = cell2mat(rawNumericColumns(:, 14));
Bx = cell2mat(rawNumericColumns(:, 15));
By = cell2mat(rawNumericColumns(:, 16));
Bz = cell2mat(rawNumericColumns(:, 17));


%% Clear temporary variables
clearvars filename delimiter startRow formatSpec fileID dataArray ans raw col numericData rawData row regexstr result numbers invalidThousandsSeparator thousandsRegExp me rawNumericColumns rawCellColumns R;