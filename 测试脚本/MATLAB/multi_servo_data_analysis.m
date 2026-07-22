%% 改版 2.0 多机 vs 原厂多机 UART 时序对比
% 数据格式：Id, Time[ns], 0:UART: RX/TX

clear; clc; close all;

%% ---------- 路径 ----------
thisDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(thisDir, '..', '..', '测试数据');
try
    dataDir = char(java.io.File(dataDir).getCanonicalPath());
catch
end

fileNew = fullfile(dataDir, '改版2.0多机测试数据--260721-103751.csv');
fileOem = fullfile(dataDir, '原厂多机测试数据--260721-104538.csv');

%% ---------- 读取 / 清洗 ----------
newData = readUartCsv(fileNew);
oemData = readUartCsv(fileOem);

cleanDir = fullfile(dataDir, 'cleaned');
if ~isfolder(cleanDir)
    mkdir(cleanDir);
end

newClean = cleanUartIntervals(newData);
oemClean = cleanUartIntervals(oemData);

fprintf('\n清洗结果:\n  %s\n  %s\n', ...
    writeUartDeltaCsv(newClean, cleanDir), ...
    writeUartDeltaCsv(oemClean, cleanDir));

% Diff = 改版 - 原厂（>0 表示改版该位置间隔更长）
cmpOut = writePairDeltaCsv(newClean, oemClean, cleanDir, ...
    'fix2_multi_vs_oem_multi_delta_compare.csv');
fprintf('对比结果:\n  %s\n', cmpOut);

%% ---------- 摘要 ----------
printUartSummary('改版2.0多机', newClean);
printUartSummary('原厂多机', oemClean);
printPairSummary(newClean, oemClean, '改版2.0多机', '原厂多机');

%% ---------- 图像 ----------
figDir = fullfile(cleanDir, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end
plotPairFigures(newClean, oemClean, figDir, 'Fix2.0-Multi', 'OEM-Multi', 'fix2_multi_vs_oem_multi');
fprintf('\n对比图已写入:\n  %s\n', figDir);

%% ========================================================================
function data = readUartCsv(csvPath)
    if ~isfile(csvPath)
        error('文件不存在: %s', csvPath);
    end
    opts = detectImportOptions(csvPath, 'NumHeaderLines', 0);
    opts.VariableNamingRule = 'preserve';
    opts = setvartype(opts, opts.VariableNames{3}, 'char');
    T = readtable(csvPath, opts);
    if width(T) < 3
        error('CSV 列数不足 3: %s', csvPath);
    end
    id = T{:, 1};
    time_ns = T{:, 2};
    hexStr = normalizeHexColumn(T{:, 3});
    valid = ~cellfun(@isempty, hexStr);
    if ~all(valid)
        warning('%s: 丢弃 %d 行空 UART 值', csvPath, sum(~valid));
        id = id(valid); time_ns = time_ns(valid); hexStr = hexStr(valid);
    end
    data = struct();
    data.file = csvPath;
    [~, name, ext] = fileparts(csvPath);
    data.name = [name, ext];
    data.id = id(:);
    data.time_ns = double(time_ns(:));
    data.time_us = data.time_ns / 1e3;
    data.time_ms = data.time_ns / 1e6;
    data.hex = hexStr(:);
    data.byte = uint8(hex2dec(hexStr));
    data.n = numel(data.byte);
end

function hexStr = normalizeHexColumn(rawHex)
    if isnumeric(rawHex)
        hexStr = arrayfun(@(x) upper(sprintf('%02X', x)), rawHex, 'UniformOutput', false);
    elseif isstring(rawHex)
        hexStr = cellstr(rawHex);
    elseif ischar(rawHex)
        hexStr = cellstr(rawHex);
    elseif iscell(rawHex)
        hexStr = rawHex;
    else
        hexStr = cellstr(string(rawHex));
    end
    hexStr = cellfun(@upper, strtrim(hexStr), 'UniformOutput', false);
end

function data = cleanUartIntervals(data)
    data.delta_ns = zeros(data.n, 1);
    if data.n > 1
        data.delta_ns(2:end) = diff(data.time_ns);
    end
    data.delta_us = data.delta_ns / 1e3;
    data.delta_ms = data.delta_ns / 1e6;
end

function outPath = writeUartDeltaCsv(data, outDir)
    if ~isfolder(outDir), mkdir(outDir); end
    [~, baseName, ext] = fileparts(data.file);
    outPath = absPath(fullfile(outDir, [baseName, '_delta', ext]));
    [fid, outPath] = openWriteFid(outPath);
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, 'Id,DeltaTime[ns],0:UART: RX/TX\n');
    for i = 1:data.n
        fprintf(fid, '%d,%.2f,%s\n', data.id(i), data.delta_ns(i), data.hex{i});
    end
end

function outPath = writePairDeltaCsv(a, b, outDir, fileName)
%WRITEPAIRDELTACSV Diff = a - b（第一组减第二组）
    if ~isfolder(outDir), mkdir(outDir); end
    n = min(a.n, b.n);
    if a.n ~= b.n
        warning('字节数不同：A=%d B=%d，按前 %d 行对齐', a.n, b.n, n);
    end
    outPath = absPath(fullfile(outDir, fileName));
    [fid, outPath] = openWriteFid(outPath);
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, ['Id,AHex,BHex,ADelta[ns],BDelta[ns],' ...
        'AMinusB[ns],SameByte\n']);
    for i = 1:n
        dA = a.delta_ns(i);
        dB = b.delta_ns(i);
        fprintf(fid, '%d,%s,%s,%.2f,%.2f,%.2f,%d\n', ...
            i, a.hex{i}, b.hex{i}, dA, dB, dA - dB, ...
            strcmp(a.hex{i}, b.hex{i}));
    end
end

function p = absPath(p)
    try
        p = char(java.io.File(p).getCanonicalPath());
    catch
        if isempty(regexp(p, '^[A-Za-z]:[\\/]', 'once'))
            p = fullfile(pwd, p);
        end
    end
end

function [fid, outPath] = openWriteFid(outPath)
    fid = fopen(outPath, 'w');
    if fid >= 0, return; end
    [folder, name, ext] = fileparts(outPath);
    altPath = fullfile(folder, [name, '_new', ext]);
    warning('无法覆盖原文件，改写到:\n  %s', altPath);
    fid = fopen(altPath, 'w');
    if fid < 0
        error('无法写入 CSV: %s', outPath);
    end
    outPath = altPath;
end

function printUartSummary(tag, data)
    dt_us = data.delta_us(2:end);
    fprintf('\n===== %s: %s =====\n', tag, data.name);
    fprintf('  字节数     : %d\n', data.n);
    fprintf('  总时长     : %.3f ms\n', sum(data.delta_ns) / 1e6);
    if ~isempty(dt_us)
        fprintf('  相邻间隔us : min=%.2f  median=%.2f  max=%.2f\n', ...
            min(dt_us), median(dt_us), max(dt_us));
    end
    fprintf('  前 16 字节 : %s\n', sprintf('%02X ', data.byte(1:min(16, data.n))));
end

function printPairSummary(a, b, nameA, nameB)
    n = min(a.n, b.n);
    fprintf('\n===== %s vs %s (n=%d) =====\n', nameA, nameB, n);
    printTurnStats([nameA ' TX->RX'], extractTurnaroundsUs(a));
    printTurnStats([nameB ' TX->RX'], extractTurnaroundsUs(b));
    printRespIntra(nameA, a);
    printRespIntra(nameB, b);

    tA = extractTurnaroundsUs(a);
    tB = extractTurnaroundsUs(b);
    m = min(numel(tA), numel(tB));
    if m > 0
        d = tA(1:m) - tB(1:m);
        fprintf('  转化差(%s-%s) us: p50=%.2f mean=%.2f\n', ...
            nameA, nameB, median(d), mean(d));
    end
end

function printRespIntra(tag, data)
    pkts = assembleStsPackets(data);
    g = [];
    for i = 1:numel(pkts)
        if pkts(i).kind == "resp"
            idx = (pkts(i).startIdx + 1):pkts(i).endIdx;
            g = [g; data.delta_us(idx)]; %#ok<AGROW>
        end
    end
    if isempty(g)
        fprintf('  %s RESP intra: 无\n', tag);
    else
        fprintf('  %s RESP intra: p50=%.2f mean=%.2f\n', tag, median(g), mean(g));
    end
end

function printTurnStats(tag, v)
    if isempty(v)
        fprintf('  %s: 无样本\n', tag); return;
    end
    fprintf('  %s: n=%d min=%.2f p50=%.2f mean=%.2f max=%.2f\n', ...
        tag, numel(v), min(v), median(v), mean(v), max(v));
end

function plotPairFigures(a, b, figDir, labelA, labelB, filePrefix)
    n = min(a.n, b.n);
    ids = (1:n)';
    dA = a.delta_us(1:n);
    dB = b.delta_us(1:n);
    cA = [0.12 0.47 0.71];
    cB = [0.84 0.15 0.16];

    fig = figure('Name', ['overview_' filePrefix], 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [80 80 1100 720]);
    subplot(3, 1, 1);
    semilogy(ids(2:end), dA(2:end), 'Color', cA, 'LineWidth', 0.8); hold on;
    semilogy(ids(2:end), dB(2:end), 'Color', cB, 'LineWidth', 0.8);
    grid on; ylabel('Delta (us)');
    title(sprintf('Byte interval: %s vs %s (log y)', labelA, labelB));
    legend({labelA, labelB}, 'Location', 'northeast');

    subplot(3, 1, 2);
    dd = dA - dB; mask = abs(dd) <= 1000;
    plot(ids(mask), dd(mask), 'Color', [0.17 0.63 0.17], 'LineWidth', 0.7); hold on;
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; ylabel('Diff (us)');
    title(sprintf('%s - %s (|Diff|<=1000 us)', labelA, labelB));

    subplot(3, 1, 3);
    plot(ids(2:end), dd(2:end), 'Color', [0.58 0.40 0.74], 'LineWidth', 0.7); hold on;
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; ylabel('Diff (us)'); xlabel('Byte index');
    title(sprintf('%s - %s (full)', labelA, labelB));
    saveFig(fig, fullfile(figDir, ['01_' filePrefix '_overview.png']));

    mask = (dA < 20) & (dB < 20) & (ids > 1);
    fig = figure('Name', ['intra_' filePrefix], 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [100 100 1100 480]);
    plot(ids(mask), dA(mask), '-', 'Color', cA, 'LineWidth', 0.8); hold on;
    plot(ids(mask), dB(mask), '-', 'Color', cB, 'LineWidth', 0.8);
    grid on; ylabel('Delta (us)'); xlabel('Byte index');
    title('Intra-frame (<20 us)');
    legend({labelA, labelB}, 'Location', 'best');
    saveFig(fig, fullfile(figDir, ['02_' filePrefix '_intra.png']));

    tA = extractTurnaroundsUs(a);
    tB = extractTurnaroundsUs(b);
    m = min(numel(tA), numel(tB));
    x = (1:m)';
    fig = figure('Name', ['turnaround_' filePrefix], 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [120 100 1000 560]);
    subplot(2, 1, 1);
    plot(x, tA(1:m), 'o-', 'Color', cA, 'LineWidth', 1); hold on;
    plot(x, tB(1:m), 's-', 'Color', cB, 'LineWidth', 1);
    grid on; ylabel('Turnaround (us)');
    title(sprintf('TX->RX turnaround: %s vs %s', labelA, labelB));
    legend({labelA, labelB}, 'Location', 'best');
    subplot(2, 1, 2);
    d = tA(1:m) - tB(1:m);
    bar(x, d, 0.7, 'FaceColor', [0.90 0.45 0.05]); hold on;
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; xlabel('Response index'); ylabel('Diff (us)');
    title(sprintf('Turnaround Diff = %s - %s', labelA, labelB));
    saveFig(fig, fullfile(figDir, ['03_' filePrefix '_turnaround.png']));

    labels = {'min', 'p50', 'mean', 'p95', 'max'};
    sA = localStats(tA); sB = localStats(tB);
    fig = figure('Name', ['turnaround_bars_' filePrefix], 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [140 120 780 400]);
    xb = 1:numel(labels); w = 0.35;
    bar(xb - w/2, sA, w, 'FaceColor', cA); hold on;
    bar(xb + w/2, sB, w, 'FaceColor', cB);
    set(gca, 'XTick', xb, 'XTickLabel', labels);
    ylabel('us');
    title(sprintf('Turnaround summary: %s vs %s', labelA, labelB));
    legend({labelA, labelB}, 'Location', 'northwest');
    grid on;
    for i = 1:numel(labels)
        text(xb(i) - w/2, sA(i), sprintf('%.1f', sA(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
        text(xb(i) + w/2, sB(i), sprintf('%.1f', sB(i)), ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'FontSize', 8);
    end
    saveFig(fig, fullfile(figDir, ['04_' filePrefix '_turnaround_bars.png']));

    [rqA, rsA] = frameDurationsUs(a);
    [rqB, rsB] = frameDurationsUs(b);
    fig = figure('Name', ['frame_dur_' filePrefix], 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [160 140 720 400]);
    vals = [median(rqA), median(rqB); median(rsA), median(rsB)];
    bb = bar(vals);
    bb(1).FaceColor = cA; bb(2).FaceColor = cB;
    set(gca, 'XTickLabel', {'Request', 'Response'});
    ylabel('Frame duration median (us)');
    title('Frame duration (intra)');
    legend({labelA, labelB}, 'Location', 'best');
    grid on;
    saveFig(fig, fullfile(figDir, ['05_' filePrefix '_frame_duration.png']));
end

function [reqDur, respDur] = frameDurationsUs(data)
    pkts = assembleStsPackets(data);
    reqDur = []; respDur = [];
    for i = 1:numel(pkts)
        idx = (pkts(i).startIdx + 1):pkts(i).endIdx;
        dur = sum(data.delta_us(idx));
        if pkts(i).kind == "req"
            reqDur(end+1, 1) = dur; %#ok<AGROW>
        else
            respDur(end+1, 1) = dur; %#ok<AGROW>
        end
    end
end

function s = localStats(v)
    if isempty(v), s = nan(1, 5); return; end
    v = sort(v(:)); n = numel(v);
    s = [min(v), median(v), mean(v), v(max(1, round(0.95*(n-1))+1)), max(v)];
end

function turn_us = extractTurnaroundsUs(data)
    pkts = assembleStsPackets(data);
    turn_us = [];
    for i = 1:numel(pkts)
        if pkts(i).kind == "resp"
            turn_us(end+1, 1) = data.delta_us(pkts(i).startIdx); %#ok<AGROW>
        end
    end
end

function pkts = assembleStsPackets(data)
    pkts = struct('startIdx', {}, 'endIdx', {}, 'kind', {}, 'len', {});
    i = 1; n = data.n;
    while i <= n
        if i + 3 > n, break; end
        if data.byte(i) ~= 255 || data.byte(i + 1) ~= 255
            i = i + 1; continue;
        end
        lenB = double(data.byte(i + 3));
        last = i + 3 + lenB;
        if last > n, break; end
        if lenB == hex2dec('13'), kind = "resp"; else, kind = "req"; end
        pkts(end+1) = struct('startIdx', i, 'endIdx', last, 'kind', kind, 'len', lenB); %#ok<AGROW>
        i = last + 1;
    end
end

function saveFig(fig, outPath)
    try
        exportgraphics(fig, outPath, 'Resolution', 150);
    catch
        print(fig, outPath, '-dpng', '-r150');
    end
    fprintf('  wrote %s\n', outPath);
end
