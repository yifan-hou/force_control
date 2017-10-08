egm_data = dlmread('../data/egm_data.txt');
ati_data = dlmread('../data/netft_data.txt');


egm.r.time = egm_data(:, 1);
egm.r.x    = egm_data(:, 2);
egm.r.y    = egm_data(:, 3);
egm.r.z    = egm_data(:, 4);
egm.r.q0   = egm_data(:, 5);
egm.r.q1   = egm_data(:, 6);
egm.r.q2   = egm_data(:, 7);
egm.r.q3   = egm_data(:, 8);
egm.s.time = egm_data(:, 9);
egm.s.x    = egm_data(:, 10);
egm.s.y    = egm_data(:, 11);
egm.s.z    = egm_data(:, 12);
egm.s.q0   = egm_data(:, 13);
egm.s.q1   = egm_data(:, 14);
egm.s.q2   = egm_data(:, 15);
egm.s.q3   = egm_data(:, 16);

ati.time         = ati_data(:, 1);
idx              = ati.time < egm.r.time(1);
ati_data(idx, :) = [];

ati.time = ati_data(:, 1);
ati.fx   = ati_data(:, 2);
ati.fy   = ati_data(:, 3);
ati.fz   = ati_data(:, 4);
ati.tx   = ati_data(:, 5);
ati.ty   = ati_data(:, 6);
ati.tz   = ati_data(:, 7);

time_offset = egm.r.time(1);
egm.r.time  = egm.r.time - time_offset;
egm.s.time  = egm.s.time - time_offset;
ati.time    = ati.time - time_offset;

figure(1);clf(1);hold on;
plot(egm.r.time, egm.r.z, '. - b', 'linewidth', 1.5);
plot(egm.s.time, egm.s.z, '. - r', 'linewidth', 1.5);
% plot(ati.time, ati.fx, '-');
% plot(ati.time, ati.fy, '-');
plot(ati.time, ati.fz + 380, '-');
% plot(ati.time, ati.tx, '-');
% plot(ati.time, ati.ty, '-');
% plot(ati.time, ati.tz, '-');

