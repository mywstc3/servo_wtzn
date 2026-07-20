%% 多机数据分析 —— 读取逻辑分析仪 UART 导出 CSV
% 数据格式（逻辑分析仪 decoder 导出）：
%   Id, Time[ns], 0:UART: RX/TX
%   1,  32413050.00, FF
%   2,  32424000.00, FF
%   ...
% 第三列为十六进制字节字符串；截图中左右错列仅为显示着色，CSV 实为单列。

clear; clc; close all;

%% ---------- 路径 ----------
thisDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(thisDir, '..', '..', '测试数据');

fileSingle = fullfile(dataDir, '第二次单机测试数据--260720-115643.csv');
fileMulti  = fullfile(dataDir, '第二次多机测试数据--260720-114529.csv');

%% ---------- 读取 ----------
singleData = readUartCsv(fileSingle);
multiData  = readUartCsv(fileMulti);

%% ---------- 数据清洗：绝对时间 -> 相邻字节间隔 ----------
cleanDir = fullfile(dataDir, 'cleaned');
if ~isfolder(cleanDir)
    mkdir(cleanDir);
end

singleClean = cleanUartIntervals(singleData);
multiClean  = cleanUartIntervals(multiData);

singleOut = writeUartDeltaCsv(singleClean, cleanDir);
multiOut  = writeUartDeltaCsv(multiClean, cleanDir);

fprintf('\n清洗结果已写入（未覆盖原文件）:\n  %s\n  %s\n', singleOut, multiOut);

%% ---------- 对比：对应序号上 多机Δt - 单机Δt（字节值可不相同）----------
cmpOut = writeDeltaDiffCsv(singleClean, multiClean, cleanDir);
fprintf('对比结果已写入:\n  %s\n', cmpOut);

%% ---------- 摘要 ----------
printUartSummary('单机-原始', singleData);
printUartSummary('单机-清洗', singleClean);
printUartSummary('多机-原始', multiData);
printUartSummary('多机-清洗', multiClean);
printDeltaDiffSummary(singleClean, multiClean);

%% ========================================================================
%  本地函数
% ========================================================================

function data = readUartCsv(csvPath)
%READUARTCSV 读取逻辑分析仪 UART CSV，返回结构体
%   data.file     - 文件路径
%   data.name     - 文件名
%   data.id       - 行号 (N x 1)
%   data.time_ns  - 时间戳，纳秒 (N x 1)
%   data.time_us  - 时间戳，微秒 (N x 1)
%   data.time_ms  - 时间戳，毫秒 (N x 1)
%   data.byte     - 字节数值 uint8 (N x 1)
%   data.hex      - 原始十六进制字符串 cellstr (N x 1)
%   data.n        - 字节数

    if ~isfile(csvPath)
        error('文件不存在: %s', csvPath);
    end

    % UART 列形如 FF/02/AE，detectImportOptions 常会误判为数值，强制按文本读
    opts = detectImportOptions(csvPath, 'NumHeaderLines', 0);
    opts.VariableNamingRule = 'preserve';
    uartColName = opts.VariableNames{3};
    opts = setvartype(opts, uartColName, 'char');

    T = readtable(csvPath, opts);

    if width(T) < 3
        error('CSV 列数不足 3（Id / Time[ns] / UART）: %s', csvPath);
    end

    id = T{:, 1};
    time_ns = T{:, 2};
    hexStr = normalizeHexColumn(T{:, 3});
    valid = ~cellfun(@isempty, hexStr);
    if ~all(valid)
        warning('%s: 丢弃 %d 行空 UART 值', csvPath, sum(~valid));
        id = id(valid);
        time_ns = time_ns(valid);
        hexStr = hexStr(valid);
    end

    byteVals = uint8(hex2dec(hexStr));

    data = struct();
    data.file    = csvPath;
    data.name    = csvPath;
    [~, name, ext] = fileparts(csvPath);
    data.name    = [name, ext];
    data.id      = id(:);
    data.time_ns = double(time_ns(:));
    data.time_us = data.time_ns / 1e3;
    data.time_ms = data.time_ns / 1e6;
    data.byte    = byteVals(:);
    data.hex     = hexStr(:);
    data.n       = numel(data.byte);
end

function hexStr = normalizeHexColumn(rawHex)
%NORMALIZEHEXCOLUMN 将 UART 列统一转为去空白后的十六进制 cellstr
    if isnumeric(rawHex)
        hexStr = arrayfun(@(x) upper(sprintf('%02X', x)), rawHex, ...
            'UniformOutput', false);
    elseif isstring(rawHex)
        hexStr = cellstr(rawHex);
    elseif ischar(rawHex)
        hexStr = cellstr(rawHex);
    elseif iscell(rawHex)
        hexStr = rawHex;
    else
        hexStr = cellstr(string(rawHex));
    end

    hexStr = strtrim(hexStr);
    hexStr = cellfun(@upper, hexStr, 'UniformOutput', false);
end

function data = cleanUartIntervals(data)
%CLEANUARTINTERVALS 将绝对时间戳转为相对上一字节的时间间隔
%   第 1 行无上一字节，DeltaTime 记为 0
    data.delta_ns = zeros(data.n, 1);
    if data.n > 1
        data.delta_ns(2:end) = diff(data.time_ns);
    end
    data.delta_us = data.delta_ns / 1e3;
    data.delta_ms = data.delta_ns / 1e6;
end

function outPath = writeUartDeltaCsv(data, outDir)
%WRITEUARTDELTACSV 写出清洗后的 CSV，Time 列改为 DeltaTime[ns]
    [~, baseName, ext] = fileparts(data.file);
    outPath = fullfile(outDir, [baseName, '_delta', ext]);

    fid = fopen(outPath, 'w', 'n', 'UTF-8');
    if fid < 0
        error('无法写入文件: %s', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));

    fprintf(fid, 'Id,DeltaTime[ns],0:UART: RX/TX\n');
    for i = 1:data.n
        fprintf(fid, '%d,%.2f,%s\n', data.id(i), data.delta_ns(i), data.hex{i});
    end
end

function outPath = writeDeltaDiffCsv(singleData, multiData, outDir)
%WRITEDELTADIFFCSV 按序号对齐，写出 多机Δt - 单机Δt
%   字节值可不相同，仅按第 i 个字节位置对齐
    n = min(singleData.n, multiData.n);
    if singleData.n ~= multiData.n
        warning('字节数不同：单机=%d 多机=%d，按前 %d 行对齐', ...
            singleData.n, multiData.n, n);
    end

    outPath = fullfile(outDir, 'single_vs_multi_delta_diff.csv');
    fid = fopen(outPath, 'w', 'n', 'UTF-8');
    if fid < 0
        error('无法写入文件: %s', outPath);
    end
    cleaner = onCleanup(@() fclose(fid));

    % DiffDelta[ns] = 多机间隔 - 单机间隔（>0 表示多机该字节间隔更长）
    fprintf(fid, ['Id,SingleHex,MultiHex,SingleDelta[ns],MultiDelta[ns],' ...
        'DiffDelta[ns],SameByte\n']);
    for i = 1:n
        dSingle = singleData.delta_ns(i);
        dMulti  = multiData.delta_ns(i);
        dDiff   = dMulti - dSingle;
        sameByte = strcmp(singleData.hex{i}, multiData.hex{i});
        fprintf(fid, '%d,%s,%s,%.2f,%.2f,%.2f,%d\n', ...
            i, singleData.hex{i}, multiData.hex{i}, ...
            dSingle, dMulti, dDiff, sameByte);
    end
end

function printDeltaDiffSummary(singleData, multiData)
    n = min(singleData.n, multiData.n);
    dDiff = multiData.delta_ns(1:n) - singleData.delta_ns(1:n);
    % 第 1 行间隔为 0，统计从第 2 行起
    if n >= 2
        d = dDiff(2:end);
    else
        d = dDiff;
    end
    sameCnt = sum(strcmp(singleData.hex(1:n), multiData.hex(1:n)));

    fprintf('\n===== 单机 vs 多机 间隔差 (多机-单机) =====\n');
    fprintf('  对齐行数   : %d  (字节相同 %d / %d)\n', n, sameCnt, n);
    if ~isempty(d)
        fprintf('  DiffDelta us: min=%.2f  median=%.2f  mean=%.2f  max=%.2f\n', ...
            min(d)/1e3, median(d)/1e3, mean(d)/1e3, max(d)/1e3);
        fprintf('  多机更慢(>0): %d 行  多机更快(<0): %d 行\n', ...
            sum(d > 0), sum(d < 0));
    end
end

function printUartSummary(tag, data)
    if isfield(data, 'delta_us')
        dt_us = data.delta_us(2:end);
    else
        dt_us = diff(data.time_us);
    end
    fprintf('\n===== %s: %s =====\n', tag, data.name);
    fprintf('  字节数     : %d\n', data.n);
    if isfield(data, 'delta_ns')
        fprintf('  总时长     : %.3f ms\n', sum(data.delta_ns) / 1e6);
    else
        fprintf('  时间跨度   : %.3f ms  (%.3f ~ %.3f ms)\n', ...
            (data.time_ns(end) - data.time_ns(1)) / 1e6, ...
            data.time_ms(1), data.time_ms(end));
    end
    if ~isempty(dt_us)
        fprintf('  相邻间隔us : min=%.2f  median=%.2f  max=%.2f\n', ...
            min(dt_us), median(dt_us), max(dt_us));
    end
    preview = data.byte(1:min(16, data.n));
    fprintf('  前 16 字节 : %s\n', sprintf('%02X ', preview));
end
