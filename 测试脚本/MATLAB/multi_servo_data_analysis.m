%% 四组单机 UART 时序对比：自研旧版 / 改版后 / 原厂 / 虚拟舵机
% 数据格式（逻辑分析仪 decoder 导出）：
%   Id, Time[ns], 0:UART: RX/TX

clear; clc; close all;

%% ---------- 路径 ----------
thisDir = fileparts(mfilename('fullpath'));
dataDir = fullfile(thisDir, '..', '..', '测试数据');
try
    dataDir = char(java.io.File(dataDir).getCanonicalPath());
catch
end

fileOld  = fullfile(dataDir, '第二次单机测试数据--260720-115643.csv');
fileFix  = fullfile(dataDir, '改版后单机测试数据--260720-214407.csv');
fileOem  = fullfile(dataDir, '单机原厂测试数据--260720-195540.csv');
fileVirt = fullfile(dataDir, '单机虚拟舵机测试数据--260720-201735.csv');

%% ---------- 读取 ----------
oldData  = readUartCsv(fileOld);
fixData  = readUartCsv(fileFix);
oemData  = readUartCsv(fileOem);
virtData = readUartCsv(fileVirt);

%% ---------- 清洗 ----------
cleanDir = fullfile(dataDir, 'cleaned');
if ~isfolder(cleanDir)
    mkdir(cleanDir);
end

oldClean  = cleanUartIntervals(oldData);
fixClean  = cleanUartIntervals(fixData);
oemClean  = cleanUartIntervals(oemData);
virtClean = cleanUartIntervals(virtData);

fprintf('\n清洗结果:\n  %s\n  %s\n  %s\n  %s\n', ...
    writeUartDeltaCsv(oldClean, cleanDir), ...
    writeUartDeltaCsv(fixClean, cleanDir), ...
    writeUartDeltaCsv(oemClean, cleanDir), ...
    writeUartDeltaCsv(virtClean, cleanDir));

%% ---------- 四组对齐 CSV ----------
cmpOut = writeQuadDeltaCsv(oldClean, fixClean, oemClean, virtClean, cleanDir);
fprintf('四组对比结果:\n  %s\n', cmpOut);

%% ---------- 摘要 ----------
printUartSummary('自研旧版', oldClean);
printUartSummary('自研改版', fixClean);
printUartSummary('原厂单机', oemClean);
printUartSummary('虚拟舵机', virtClean);
printQuadSummary(oldClean, fixClean, oemClean, virtClean);

%% ---------- 图像 ----------
figDir = fullfile(cleanDir, 'figures');
if ~isfolder(figDir)
    mkdir(figDir);
end
plotQuadFigures(oldClean, fixClean, oemClean, virtClean, figDir);
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

function outPath = writeQuadDeltaCsv(oldD, fixD, oem, virt, outDir)
    if ~isfolder(outDir), mkdir(outDir); end
    n = min([oldD.n, fixD.n, oem.n, virt.n]);
    outPath = absPath(fullfile(outDir, 'old_fix_oem_virt_delta_compare.csv'));
    [fid, outPath] = openWriteFid(outPath);
    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
    fprintf(fid, ['Id,OldHex,FixHex,OemHex,VirtHex,' ...
        'OldDelta[ns],FixDelta[ns],OemDelta[ns],VirtDelta[ns],' ...
        'FixMinusOld[ns],OemMinusFix[ns],VirtMinusFix[ns]\n']);
    for i = 1:n
        dOld = oldD.delta_ns(i); dFix = fixD.delta_ns(i);
        dOem = oem.delta_ns(i);  dVirt = virt.delta_ns(i);
        fprintf(fid, '%d,%s,%s,%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n', ...
            i, oldD.hex{i}, fixD.hex{i}, oem.hex{i}, virt.hex{i}, ...
            dOld, dFix, dOem, dVirt, dFix - dOld, dOem - dFix, dVirt - dFix);
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

function printQuadSummary(oldD, fixD, oem, virt)
    n = min([oldD.n, fixD.n, oem.n, virt.n]);
    fprintf('\n===== 四组对齐摘要 (n=%d) =====\n', n);
    printTurnStats('旧版 TX->RX', extractTurnaroundsUs(oldD));
    printTurnStats('改版 TX->RX', extractTurnaroundsUs(fixD));
    printTurnStats('原厂 TX->RX', extractTurnaroundsUs(oem));
    printTurnStats('虚拟 TX->RX', extractTurnaroundsUs(virt));

    mask = (oldD.delta_us(1:n) < 20) & (fixD.delta_us(1:n) < 20) & ...
        (oem.delta_us(1:n) < 20) & (virt.delta_us(1:n) < 20) & ((1:n)' > 1);
    if any(mask)
        fprintf('  帧内字节间隔us (<20):\n');
        fprintf('    旧版 median=%.2f\n', median(oldD.delta_us(mask)));
        fprintf('    改版 median=%.2f\n', median(fixD.delta_us(mask)));
        fprintf('    原厂 median=%.2f\n', median(oem.delta_us(mask)));
        fprintf('    虚拟 median=%.2f\n', median(virt.delta_us(mask)));
    end

    % 应答帧内间隔（更贴近波形结论）
    printRespIntra('旧版', oldD);
    printRespIntra('改版', fixD);
    printRespIntra('原厂', oem);
    printRespIntra('虚拟', virt);
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

function plotQuadFigures(oldD, fixD, oem, virt, figDir)
    n = min([oldD.n, fixD.n, oem.n, virt.n]);
    ids = (1:n)';
    dOld = oldD.delta_us(1:n);
    dFix = fixD.delta_us(1:n);
    dOem = oem.delta_us(1:n);
    dVirt = virt.delta_us(1:n);

    cOld  = [0.50 0.50 0.50];  % gray old
    cFix  = [0.12 0.47 0.71];  % blue fixed
    cOem  = [0.84 0.15 0.16];  % red oem
    cVirt = [0.17 0.63 0.17];  % green virt

    % 1 overview
    fig = figure('Name', 'overview_4', 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [60 60 1100 780]);
    subplot(3, 1, 1);
    semilogy(ids(2:end), dOld(2:end), 'Color', cOld, 'LineWidth', 0.7); hold on;
    semilogy(ids(2:end), dFix(2:end), 'Color', cFix, 'LineWidth', 0.8);
    semilogy(ids(2:end), dOem(2:end), 'Color', cOem, 'LineWidth', 0.8);
    semilogy(ids(2:end), dVirt(2:end), 'Color', cVirt, 'LineWidth', 0.8);
    grid on; ylabel('Delta (us)');
    title('Byte interval: Old / Fixed / OEM / Virtual');
    legend({'Old', 'Fixed', 'OEM', 'Virtual'}, 'Location', 'northeast');

    subplot(3, 1, 2);
    d1 = dFix - dOld; mask = abs(d1) <= 1000;
    plot(ids(mask), d1(mask), 'Color', cFix, 'LineWidth', 0.7); hold on;
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; ylabel('Diff (us)'); title('Fixed - Old (|Diff|<=1000)');

    subplot(3, 1, 3);
    d2 = dFix - dOem; mask2 = abs(d2) <= 1000;
    plot(ids(mask2), d2(mask2), 'Color', [0.90 0.45 0.05], 'LineWidth', 0.7); hold on;
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; ylabel('Diff (us)'); xlabel('Byte index');
    title('Fixed - OEM (|Diff|<=1000)');
    saveFig(fig, fullfile(figDir, '01_quad_overview.png'));

    % 2 intra
    mask = (dOld < 20) & (dFix < 20) & (dOem < 20) & (dVirt < 20) & (ids > 1);
    fig = figure('Name', 'intra_4', 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [80 80 1100 480]);
    plot(ids(mask), dOld(mask), '-', 'Color', cOld, 'LineWidth', 0.6); hold on;
    plot(ids(mask), dFix(mask), '-', 'Color', cFix, 'LineWidth', 0.8);
    plot(ids(mask), dOem(mask), '-', 'Color', cOem, 'LineWidth', 0.8);
    plot(ids(mask), dVirt(mask), '-', 'Color', cVirt, 'LineWidth', 0.8);
    grid on; ylabel('Delta (us)'); xlabel('Byte index');
    title('Intra-frame (<20 us)');
    legend({'Old', 'Fixed', 'OEM', 'Virtual'}, 'Location', 'best');
    saveFig(fig, fullfile(figDir, '02_quad_intra.png'));

    % 3 turnaround
    tOld = extractTurnaroundsUs(oldD);
    tFix = extractTurnaroundsUs(fixD);
    tOem = extractTurnaroundsUs(oem);
    tVirt = extractTurnaroundsUs(virt);
    m = min([numel(tOld), numel(tFix), numel(tOem), numel(tVirt)]);
    x = (1:m)';
    fig = figure('Name', 'turnaround_4', 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [100 80 1000 560]);
    subplot(2, 1, 1);
    plot(x, tOld(1:m), 'x-', 'Color', cOld, 'LineWidth', 1); hold on;
    plot(x, tFix(1:m), 'o-', 'Color', cFix, 'LineWidth', 1);
    plot(x, tOem(1:m), 's-', 'Color', cOem, 'LineWidth', 1);
    plot(x, tVirt(1:m), 'd-', 'Color', cVirt, 'LineWidth', 1);
    grid on; ylabel('Turnaround (us)');
    title('TX->RX turnaround');
    legend({'Old', 'Fixed', 'OEM', 'Virtual'}, 'Location', 'best');
    subplot(2, 1, 2);
    plot(x, tFix(1:m) - tOld(1:m), 'o-', 'Color', cFix, 'LineWidth', 1); hold on;
    plot(x, tFix(1:m) - tOem(1:m), 's-', 'Color', [0.90 0.45 0.05], 'LineWidth', 1);
    yline(0, 'Color', [0.3 0.3 0.3]);
    grid on; xlabel('Response index'); ylabel('Diff (us)');
    title('Fixed vs Old / OEM');
    legend({'Fixed-Old', 'Fixed-OEM'}, 'Location', 'best');
    saveFig(fig, fullfile(figDir, '03_quad_turnaround.png'));

    % 4 bars
    labels = {'min', 'p50', 'mean', 'p95', 'max'};
    sOld = localStats(tOld); sFix = localStats(tFix);
    sOem = localStats(tOem); sVirt = localStats(tVirt);
    fig = figure('Name', 'turnaround_bars_4', 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [120 100 900 420]);
    xb = 1:numel(labels); w = 0.18;
    bar(xb - 1.5*w, sOld, w, 'FaceColor', cOld); hold on;
    bar(xb - 0.5*w, sFix, w, 'FaceColor', cFix);
    bar(xb + 0.5*w, sOem, w, 'FaceColor', cOem);
    bar(xb + 1.5*w, sVirt, w, 'FaceColor', cVirt);
    set(gca, 'XTick', xb, 'XTickLabel', labels);
    ylabel('us'); title('Turnaround summary');
    legend({'Old', 'Fixed', 'OEM', 'Virtual'}, 'Location', 'northwest');
    grid on;
    saveFig(fig, fullfile(figDir, '04_quad_turnaround_bars.png'));

    % 5 frame duration
    [rqO, rsO] = frameDurationsUs(oldD);
    [rqF, rsF] = frameDurationsUs(fixD);
    [rqE, rsE] = frameDurationsUs(oem);
    [rqV, rsV] = frameDurationsUs(virt);
    fig = figure('Name', 'frame_dur_4', 'Color', 'w', 'Visible', 'on');
    set(fig, 'Position', [140 120 780 400]);
    vals = [median(rqO), median(rqF), median(rqE), median(rqV); ...
            median(rsO), median(rsF), median(rsE), median(rsV)];
    b = bar(vals);
    b(1).FaceColor = cOld; b(2).FaceColor = cFix;
    b(3).FaceColor = cOem; b(4).FaceColor = cVirt;
    set(gca, 'XTickLabel', {'Request', 'Response'});
    ylabel('Frame duration median (us)');
    title('Frame duration (intra)');
    legend({'Old', 'Fixed', 'OEM', 'Virtual'}, 'Location', 'best');
    grid on;
    saveFig(fig, fullfile(figDir, '05_quad_frame_duration.png'));
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
