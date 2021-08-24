# Stepper Scheduling Analysis


At the time of writing this script, there are two routes that the Stepper class can be desired around.
>*Note, this script has since been converted into a README file (markdown format) such that it would be visible in github. The actual data used for this anaylsis is NOT stored in github as the size would be too big, hence only the README file is contained within this folder location. However, this README file can be copied into a MATLAB livescript, and it would work as required to generate similar results.
The location of the data can be modified in the first code call of this, however the source data needs to be a textfile (sourced from a PulseView image), and the textfile needs to contain the timing(s) of the interrupts of the TIM/DMA (I just set a GPIO pin ON/OFF whilst running interrupts).*




1) Use of the TIMER overflow, to control the updating of the current stepper motor (this however limits the number of steppers that the MCU can use - 1 per TIMER) - commit = [0004dc8](https://github.com/mipidisaster/miLibrary/tree/0004dc829de326176ad2ffe935d60c5842b51681) (tagged - v0.2.0)




2) Use of the DMA transmit complete, to control the updating of the current stepper motor (this would allow for maximum use of the TIMER on the MCU, such that 1 TIMER can support x number of steppers; were x = number of output capture/compare devices in the MCU) - commit = [b98f644](https://github.com/mipidisaster/miLibrary/tree/b98f6440f0b3737a9a3c6bd1ae8843c7e9513a28) (branch 'wip-\#5-stepper-improvements')


  


The differences between the two routes is one of MCU hardware utilisation, as well as processor utilisation. As the TIMER overflow takes up more of the MCU hardware, but has minimuma processor impact. The DMA route has least amount of hardware utilisation, however might increase processor utilisations; however it does allow for a level of flexibility as to the impact on the processor.




This flexibility comes from the sorting within the DMA multiple "PULSES" to transmit per interrupt (i.e. 1, 2, 3, 4, 5, 10, etc.); this obviously takes up a level of processor time. However, reduces the number of times that the interrupt would be triggered, reducing utilisation overall




The intent of this script is to see what the timing impact is between the original TIMER overflow route, and the new DMA transmit complete route.


  


To get the data used in this, a specific build was created which generated a sequence of movement requests:



   -  11 steps at 20ms per pulse 
   -  101 steps at 200us per pulse 
   -  Continual speed at 100us per pulse 
   -  Continual speed at 50ms per pulse 



This build would be re-created each time with the TIM overflow build, and then the DMA build with different array depths. The other variable introduced is optimisation of the build, going from "None", to more "-O2" and most "-O3", so as to determine how this affects the processor time between the builds.




Array depths to be anaylsed:




1, 2, 4, 5, 10, 20, ~~50~~




Data is located within folder named 'data'. Check the contents of this for each of the filenames containing the PulseView timing extract of the interrupt(s). Filenames follow the following:




XXY-Test-Depth-Optimisation:




XX = Test number - 01, 02, 03, etc.




Y = optimisation numbering 0 = unoptimised (-O0), 2 = "-O2", 3 = "-O3"




Test = TIM or DMA




Depth = Array depth (TIM = 0)




Optimisation = O1, O2, O3


# Retrieve the number of data files

```matlab:Code
data_files = dir('data');    % Get the contents of the folder "/data"
% Only capture the files which are not folders
data_files = data_files( arrayfun(@(files) ~files.isdir, data_files) );

timing_files = arrayfun(@(files) string(files.name), data_files);
test_numbers = unique(  arrayfun(@(files) TIMdatafiles(files, 1:2), timing_files)  );
number_of_optimisations = unique(  arrayfun(@(files) TIMdatafiles(files, 3), timing_files)  );
```

# Generate the histogram table, with time slices

```matlab:Code
ts          = 0.5;      % Time slices for the histogram 'buckets', in us
nbuckets    =  0:ts:80; % Buckets for the histogram(s)

median_time     = zeros(1, length(timing_files));
max_time        = zeros(1, length(timing_files));
array_size      = split(timing_files, '-'); array_size = double( array_size(:, 3) );

histogram_contents = zeros(length(nbuckets), length(timing_files));
histogram_contents = array2timetable(histogram_contents, ...
                                     'RowTimes', seconds(nbuckets), ...
                                     'VariableNames', timing_files);
```

# Cycle through the data files

```matlab:Code
for n = 1:length(data_files)
```



For each file, import the data into MATLAB - the ".data" will include all the timings from Pulseview in "ns". This data shows the time that the interrupt has been running for, as well as the time period between interrupts.




First step is to convert the "ns" into "us".



```matlab:Code
    pulseview = importdata(fullfile(data_files(n).folder, data_files(n).name));
    pulseview.data = pulseview.data / 1000;
```



Generate a histogram of the data using the slices (nbuckets), and put into the timetable.



```matlab:Code
    time_buckets = histcounts(pulseview.data, nbuckets);
    histogram_contents(:,n) = table([0, time_buckets]');
```



Determine the time where the maximum number of counts occured; this is the median of the specified test number with the desired optimisation. Store this within a variable array, for future plotting.



```matlab:Code
    [~, I] = max(histogram_contents(:,n).Variables);
    median_time(n) = seconds(  histogram_contents.Time(I)  );
    
```



Determine the highest bucket which contacts data, this is the maximum time that the interrupt has been run for (for at least a single sample). Store this within a variable array, for future plotting.



```matlab:Code
    I = histogram_contents.Time((histogram_contents(:,n).Variables ~= 0));
    max_time(n) = seconds(  I(end)  );
end
```



Now we want to display the times were the number of counts is greater than 0, and have these grouped together for the individual test numbers: "00", "01", "02", etc.




So once again cycle through the test numbers:



```matlab:Code
for n = 1:length(test_numbers)
```



For each test number, get the number of entries which are non-zero, and put into temp variable.




Then put this variable into the histogram_contents, so as to see the time slices which have occured



```matlab:Code
    [row, ~] = find(histogram_contents(:,timing_files(...
                     strncmp(timing_files, test_numbers(n), 2))).Variables ~= 0);
    disp(histogram_contents(unique(row),...
                            timing_files(strncmp(timing_files, test_numbers(n), 2))));
end
```


```text:Output
      Time      000-TIM-0-O0.txt    001-TIM-0-O1.txt    002-TIM-0-O2.txt    003-TIM-0-O3.txt    004-TIM-0-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    2.5 sec              0                1210               84947               72785                1210      
    3 sec                0               83736                   0               12162               83725      
    4 sec             1210                   0                   0                   0                   0      
    4.5 sec          81814                   0                   0                   0                   0      
    5 sec             1919                   0                   0                   0                   0      
    6.5 sec              0                   0                   0                  11                   0      
    7 sec                0                   0                  11                   0                   0      
    8 sec                0                  11                  14                  22                  11      
    8.5 sec              0                   0                   8                  10                   0      
    9.5 sec              0                   0                   7                   0                  22      
    10 sec               0                  22                   3                   0                   0      
    11 sec               0                   0                   0                   0                   5      
    11.5 sec             0                   3                   0                   0                   5      
    12 sec               0                   7                   0                   0                   0      
    14 sec              11                   0                   0                   0                   0      
    17 sec              21                   0                   0                   0                   0      
    17.5 sec             1                   0                   0                   0                   0      
    21 sec              10                   0                   0                   0                   0      

      Time      010-DMA-1-O0.txt    011-DMA-1-O1.txt    012-DMA-1-O2.txt    013-DMA-1-O3.txt    014-DMA-1-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    9.5 sec              0                   0                 683               64801                   0      
    10 sec               0                   0               81019               16901                1210      
    10.5 sec             0                   0                   0                   0               80491      
    11.5 sec             0                1210                   0                   0                   0      
    12 sec               0               80492                   0                   0                   0      
    13 sec               0                   0                   0                   6                   0      
    13.5 sec             0                   0                   0                   5                   0      
    14 sec               0                   0                   7                   0                   0      
    14.5 sec             0                   0                   4                   0                   0      
    15.5 sec             0                   0                   0                   0                  11      
    16.5 sec             0                  11                   0                  10                   0      
    18.5 sec          1210                   0                  10                   0                   0      
    19 sec               0                   0                   0                  22                   0      
    19.5 sec         80487                   0                   0                   0                   0      
    20 sec               0                   0                  19                   0                   0      
    20.5 sec             0                   0                   3                   0                  10      
    21 sec               0                  10                   0                   0                  22      
    22.5 sec             0                  22                   0                   0                   0      
    28 sec               8                   0                   0                   0                   0      
    28.5 sec             3                   0                   0                   0                   0      
    37.5 sec             9                   0                   0                   0                   0      
    38 sec               1                   0                   0                   0                   0      
    39 sec              22                   0                   0                   0                   0      

      Time      020-DMA-2-O0.txt    021-DMA-2-O1.txt    022-DMA-2-O2.txt    023-DMA-2-O3.txt    024-DMA-2-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    9.5 sec              0                   0                   3                   0                   0      
    10 sec               0                   0                 602                 111                  22      
    10.5 sec             0                   0               41231               41726                 583      
    11 sec               0                   0                   0                   0               39870      
    11.5 sec             0                 563                   0                   0                1361      
    12 sec               0               34721                   0                   0                   0      
    12.5 sec             0                6552                   0                   0                   0      
    14 sec               0                   0                   0                  11                   0      
    14.5 sec             0                   0                   6                   0                   0      
    15 sec               0                   0                   5                   0                   0      
    16 sec               0                   0                   0                   0                  11      
    16.5 sec             0                   6                   0                   0                   0      
    17 sec               0                   5                   0                   0                   0      
    17.5 sec             0                   0                   0                  10                   0      
    18.5 sec             3                   0                   0                   0                   0      
    19 sec              19                   0                  10                   0                   0      
    19.5 sec             0                   0                   0                  20                   0      
    20 sec               5                   0                   0                   2                   0      
    20.5 sec           578                   0                  22                   0                   0      
    21 sec           41229                   0                   0                   0                  10      
    21.5 sec             0                  10                   0                   0                  11      
    22 sec               0                   0                   0                   0                  11      
    23 sec               0                  22                   0                   0                   0      
    30 sec              11                   0                   0                   0                   0      
    39.5 sec            10                   0                   0                   0                   0      
    40.5 sec            18                   0                   0                   0                   0      
    41 sec               4                   0                   0                   0                   0      

      Time      030-DMA-3-O0.txt    031-DMA-3-O1.txt    032-DMA-3-O2.txt    033-DMA-3-O3.txt    034-DMA-3-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    10.5 sec             0                   0                  22                 396                   0      
    11 sec               0                   0               24812               24438                  20      
    11.5 sec             0                   0                   0                   0                 327      
    12 sec               0                  22                   0                   0               11009      
    12.5 sec             0                 374                   0                   0               11268      
    13 sec               0               24438                   0                   0                   0      
    14.5 sec             0                   0                   0                  11                   0      
    15.5 sec             0                   0                  11                   0                   0      
    17 sec               0                   0                   0                   0                  10      
    17.5 sec             0                  11                   0                   7                   0      
    18 sec               0                   0                   0                   3                   0      
    19.5 sec             0                   0                  10                   0                   0      
    20 sec               0                   0                   0                  22                   0      
    20.5 sec            22                   0                   0                   0                   0      
    21 sec               0                   0                  21                   0                   0      
    21.5 sec             0                   0                   1                   0                   0      
    22 sec             374                   4                   0                   0                   9      
    22.5 sec             0                   6                   0                   0                   0      
    23 sec           24436                   0                   0                   0                  20      
    23.5 sec             0                  14                   0                   0                   0      
    24 sec               0                   8                   0                   0                   0      
    32 sec              11                   0                   0                   0                   0      
    41 sec               7                   0                   0                   0                   0      
    41.5 sec             3                   0                   0                   0                   0      
    42.5 sec            22                   0                   0                   0                   0      

      Time      040-DMA-4-O0.txt    041-DMA-4-O1.txt    042-DMA-4-O2.txt    043-DMA-4-O3.txt    044-DMA-4-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    9.5 sec              0                   0                   0                  11                   0      
    10 sec               0                   0                  11                   0                   7      
    10.5 sec             0                   0                   0                   0                   4      
    11 sec               0                   0                   8                  11                   0      
    11.5 sec             0                  11                 278               21139                  11      
    12 sec               0                   0               20864                   0                 133      
    12.5 sec             0                   1                   0                   0                 142      
    13 sec               0                 285                   0                   0               20864      
    13.5 sec             0               20861                   0                   0                   0      
    14 sec               0                   3                   0                   0                   0      
    15 sec               0                   0                   0                  11                   0      
    16 sec               0                   0                  11                   0                   0      
    18 sec               0                   0                   0                   0                  11      
    18.5 sec             4                  11                   0                  10                   0      
    19 sec               7                   0                   0                   0                   0      
    20.5 sec             0                   0                  10                  22                   0      
    21.5 sec             0                   0                   5                   0                   0      
    22 sec               0                   0                  17                   0                   0      
    22.5 sec            11                   0                   0                   0                   0      
    23 sec               0                  10                   0                   0                  10      
    23.5 sec             0                   0                   0                   0                  11      
    24 sec             275                   2                   0                   0                  11      
    24.5 sec         20804                  20                   0                   0                   0      
    25 sec              58                   0                   0                   0                   0      
    33.5 sec            11                   0                   0                   0                   0      
    43 sec              10                   0                   0                   0                   0      
    44 sec              10                   0                   0                   0                   0      
    44.5 sec            12                   0                   0                   0                   0      

      Time      050-DMA-5-O0.txt    051-DMA-5-O1.txt    052-DMA-5-O2.txt    053-DMA-5-O3.txt    054-DMA-5-Os.txt
    ________    ________________    ________________    ________________    ________________    ________________

    9.5 sec              0                   0                   6                   0                   0      
    10 sec               0                   0                  16                  22                  22      
    11.5 sec             0                  22                   0                   0                   0      
    12 sec               0                   0                 219                   0                   0      
    12.5 sec             0                   0               16729               16948                 116      
    13 sec               0                   0                   0                   0                8404      
    13.5 sec             0                 215                   0                   0                8428      
    14 sec               0                8704                   0                   0                   0      
    14.5 sec             0                8029                   0                   0                   0      
    16 sec               0                   0                   0                  11                   0      
    16.5 sec             0                   0                   2                   0                   0      
    17 sec               0                   0                   9                   0                   0      
    18 sec               0                   0                   0                   0                  11      
    18.5 sec             4                   0                   0                   0                   0      
    19 sec              14                  11                   0                   0                   0      
    19.5 sec             0                   0                   0                  10                   0      
    21 sec               0                   0                  10                   0                   0      
    21.5 sec             0                   0                   0                  22                   0      
    22.5 sec             0                   0                  22                   0                   0      
    23 sec               0                   0                   0                   0                   9      
    23.5 sec             0                  10                   0                   0                  12      
    24 sec               0                   0                   0                   0                  11      
    25 sec               0                  22                   0                   0                   0      
    25.5 sec           180                   0                   0                   0                   0      
    26.5 sec         13681                   0                   0                   0                   0      
    35.5 sec             9                   0                   0                   0                   0      
    44.5 sec             2                   0                   0                   0                   0      
    45 sec               6                   0                   0                   0                   0      
    46 sec              18                   0                   0                   0                   0      

      Time      060-DMA-10-O0.txt    061-DMA-10-O1.txt    062-DMA-10-O2.txt    063-DMA-10-O3.txt    064-DMA-10-Os.txt
    ________    _________________    _________________    _________________    _________________    _________________

    9.5 sec              0                    0                    7                    0                    0       
    10 sec               0                    0                   15                   22                   22       
    11.5 sec             0                   20                    0                    0                    0       
    15.5 sec             0                    0                 8400                    0                    6       
    16 sec               0                    0                   95                 8495                   45       
    16.5 sec             0                   90                    0                    0                 4242       
    17 sec               0                 7075                    0                    0                 4202       
    17.5 sec             0                  559                    0                    0                    0       
    18.5 sec             7                    0                    0                    0                    0       
    19 sec              15                    0                    0                    0                    0       
    19.5 sec             0                    0                    0                   11                    0       
    20 sec               0                    0                   10                    0                    0       
    20.5 sec             0                    0                    1                    0                    0       
    21.5 sec             0                    0                    0                    0                   11       
    22 sec               0                   10                    0                    0                    0       
    23 sec               0                    0                    0                   10                    0       
    24.5 sec             0                    0                   10                    0                    0       
    25 sec               0                    0                    0                   22                    0       
    25.5 sec             0                    0                   10                    0                    0       
    26 sec               0                    0                   12                    0                    0       
    26.5 sec             0                   10                    0                    0                   10       
    27 sec               0                    0                    0                    0                   11       
    27.5 sec             0                    0                    0                    0                   11       
    28 sec               0                   20                    0                    0                    0       
    34.5 sec            99                    0                    0                    0                    0       
    35.5 sec          8396                    0                    0                    0                    0       
    44.5 sec            11                    0                    0                    0                    0       
    53.5 sec             9                    0                    0                    0                    0       
    54 sec               1                    0                    0                    0                    0       
    55 sec              22                    0                    0                    0                    0       

      Time      070-DMA-20-O0.txt    071-DMA-20-O1.txt    072-DMA-20-O2.txt    073-DMA-20-O3.txt    074-DMA-20-Os.txt
    ________    _________________    _________________    _________________    _________________    _________________

    9.5 sec              0                    0                    3                    0                    0       
    10 sec               0                    0                    8                   11                   11       
    11.5 sec             0                   11                    0                    0                    0       
    18.5 sec             6                    0                    0                    0                    0       
    19 sec               5                    0                    0                    0                    0       
    22 sec               0                    0                   29                    0                    0       
    22.5 sec             0                   44                 4213                 2656                   20       
    23 sec               0                 4157                    0                 1586                 2026       
    23.5 sec             0                   41                    0                    0                  125       
    24 sec               0                    0                    0                   10                 2071       
    25.5 sec             0                    0                   10                    0                    0       
    26.5 sec             0                    0                    0                   11                    0       
    27 sec               0                    0                   11                    0                    4       
    27.5 sec             0                   10                    0                    0                    6       
    28 sec               0                   11                    0                    0                    0       
    28.5 sec             0                    0                    0                    0                   11       
    32 sec               0                    0                    0                   22                    0       
    32.5 sec             0                    0                   22                    0                    0       
    33.5 sec             0                    6                    0                    0                   11       
    34 sec               0                   16                    0                    0                    1       
    34.5 sec             0                    0                    0                    0                    5       
    35 sec               0                    0                    0                    0                    5       
    52 sec               6                    0                    0                    0                    0       
    52.5 sec            38                    0                    0                    0                    0       
    53 sec            4157                    0                    0                    0                    0       
    53.5 sec            41                    0                    0                    0                    0       
    55.5 sec             6                    0                    0                    0                    0       
    56 sec               4                    0                    0                    0                    0       
    62 sec              10                    0                    0                    0                    0       
    62.5 sec             1                    0                    0                    0                    0       
    72.5 sec            20                    0                    0                    0                    0       
    73 sec               2                    0                    0                    0                    0       
```



Generate an alternative view of the data using a heatmap, however this will be based upon the optimised levels. So one heatmap for unoptimised, another for optimisation level 1, 2, etc.




This time loop through the optimised runs, and then capture the columns which match this optimisation



```matlab:Code
for n = 1:length(number_of_optimisations)
    tmp = find ( arrayfun(@(files) TIMdatafiles(files, 3), timing_files) ...
                                        == number_of_optimisations(n)  );
```



Generate figure plot, for the heatmap. Showing Time on the Y-axis, and Data source on the x-axis. The colour will make use of a logarithm scale; due to the numbers being spaced out sparsley, and I want to be able to see where a large majority of the data points are, as well as were the occasionally point is.



```matlab:Code
    figure();
    H = heatmap(histogram_contents(:, tmp).Variables);
    H.YData = seconds( histogram_contents.Time );
    H.XData = histogram_contents.Properties.VariableNames(tmp);
    H.ColorLimits = [1, 11.5];
    YLabels = string(nbuckets);     YLabels(mod(nbuckets, 1) ~= 0) = " ";
    H.YDisplayLabels = YLabels;
        % Change resolution of the Ydata, so as to be easier to see in output file
    
    H.ColorScaling = 'log'; %H.GridVisible = 'off';
    ylabel("Interrupt time (us)"); xlabel("Test Number (filename)")
end
```


![](/_image/analysis/Stepper-Class/figure_0.png)


![](/_image/analysis/Stepper-Class/figure_1.png)


![](/_image/analysis/Stepper-Class/figure_2.png)


![](/_image/analysis/Stepper-Class/figure_3.png)


![](/_image/analysis/Stepper-Class/figure_4.png)

# Timing with multiple pulses per interrupt (DMA)


As can be seen above, it is clear to see that the DMA interrupt takes up more time compared to the TIM version. This makes sense as the DMA interrupt needs to calculate were the number pulse needs to occur, whereas the TIM version just does basic counts.




This section will attempt to demonstrate the improvement that the DMA version can introduced which the TIM version cannot: that of allow multiple pulses per interrupt; this is because the DMA can now be an array of depth (x - number of pulses), and only once all the pulses in the DMA are completed will the interrupt run. Therefore, can reduce how oftern the interrupt is called!




The following plots will show what the time is for both non-optimised (-O0), and most optimised (-O3), and then divide this by the depth of the array. Which will be able to demonstrate that as the array increases, the "theoretical" time per pulse would reduce below the TIM version; however would be concentrated at a single time.



```matlab:Code
optimised_text = split(timing_files, '.txt');   % Remove the '.txt' from the end of the file
optimised_text = split(optimised_text(:, 1), '-');  optimised_text  = optimised_text(:,4);
```



Create an array used for looping through the optimised runs, to plot.



```matlab:Code
run_type = string(  {"O0", "O3"}  );

for n = 1:length(run_type)
    positions = strcmp(optimised_text, run_type(n));
    temp_med    = median_time(positions);  temp_max     = max_time(positions);
    temp_array  = array_size(positions)';  temp_dvid    = [1, temp_array(2:end)];
    
    figure();
    plot(temp_array, [temp_med; (temp_med ./ temp_dvid);...
                      temp_max; (temp_max ./ temp_dvid)]);
    xlabel("Array Depth");
    ylabel("Interrupt time (us)");
    
    yline(temp_med(1)); yline(temp_max(1)); 
    if (n > 1)
        yline(median_time(1), 'r--'); yline(max_time(1), 'r--'); 
    end
    
    title(  sprintf("Time benefit of DMA with optimisation -%s", run_type(n))  )
    
    if (n == 1)
        legend("Median Time", "Median Time / Array", ...
               "Max Time", "Max Time / Array", ...
               "TIM - Median", "TIM - Max");
    else
        legend("Median Time", "Median Time / Array", ...
               "Max Time", "Max Time / Array", ...
               "TIM - Median", "TIM - Max", ...
               "TIM - O0 Median", "TIM - O0 Max");
    end
end
```


![](/_image/analysis/Stepper-Class/figure_5.png)


![](/_image/analysis/Stepper-Class/figure_6.png)



This graph shows some useful information. As expected the interrupt time per pulse greatly reduces with the DMA version, and this quickly goes below the TIM's fastest time with a Array Depth of \textasciitilde{}6 -> 8. Additionally, with the -O3 optimisation, DMA array depths of less than \textasciitilde{}6 are well below the TIM's -O0 max time.


# Conclusion


The take away is that the DMA interrupt takes up more time, compared to the TIM version. Due to the extra calculations needed within the interrupt to determine were the next "x" pulses need to occur (where "x" = number of pulses per interrupt). All the plots above are just multiple ways to show this, as compared to the TIM -O0 takes \textasciitilde{}20us (max) to complete, however DMA (depth = 1) takes \textasciitilde{}40us (max) - basically double the TIM. However, DMA does provide extra hardware functionality (as mentioned above).




The last lot of plots, shows one of the extra improvements of the DMA route; that of with a larger DMA array, this can reduce the interrupt time per pulse - with depth of 20, the maximum time to complete interrupt is \textasciitilde{}73us, however per pulse this comes down to \textasciitilde{}3.7us, which is below the Median time of TIM with most optimisation!




There is a ceiling to the number of pulses that can be squeezed into a single interrupt, and this was found during the course of generating the dataset for this script: whilst running with a depth of 30 pulses, it was observed that the test sequence was not being followed correctly. This was found out to be due to the update rate of the logic (which is based upon the interrupt), so with a bigger depth, the update rate drops as well (issue would of been observed with depth of 20, however this was just on the edge of the test sequence selected). So with a slow pulse rate (say 50ms), and a depth of 100, this would mean the speed would only be updated every 5s! Now it is unlikely that this pulse speed would be selected however, it does show the issue that this introduces.




So with all that being said, I am still wanting to make use of the DMA interrupt for the Stepper class - mainly for the improved hardware utilisation, as well as the reducation in steps per pulse.




The depth of the array has been selected as **10**. The reason for this is:



   1.  At all optimisation levels, the number of pulses per interrupt just below the TIM median with -O3 optimisation. Note, DMA with -O0 it can be seen that the maximum time is still just above the TIM median, however this is acceptable. 
   \item{ The maximum time to complete interrupt is \textasciitilde{}50us (-O0), which drops to \textasciitilde{}25us with -O3. The median is \textasciitilde{}30us, and \textasciitilde{}15us. }



Due to the extra time that the interrupt takes, the fastest speed that the class can support needs to be updated. At time of writing this script is 100counts which, with 1us per count, and 200 pulses per rotation = 50rps (or 3000rpm). So as to allow upto 4 steppers to be controlled at once this will be increased to 200counts - meaning that if all the update requests happen at the same time, the maximum speed would still be correctly maintained.


# Appendix
## Function to reduce the filename to get the test number

```matlab:Code
function func_output = TIMdatafiles(datafile, range)
    t = char(datafile);
    func_output = string(t(range));
end
```

