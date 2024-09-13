function helical_3_muscle_7_18_scene_plotter
    dataset_obj = load_helical_3_muscle_7_18;

    fig = uifigure;
    fig.Name = "Helical Scene Plotter";

    gridlayout = uigridlayout(fig, [2, 2]);
    gridlayout.RowHeight = {30, "1x"};
    gridlayout.ColumnWidth = {"fit", "1x"};

    lbl_pressure = uilabel(gridlayout);
    dd_measurement = uidropdown(gridlayout);
    ax = uiaxes(gridlayout);

    % Position label
    lbl_pressure.Layout.Row = 1;
    lbl_pressure.Layout.Column = 1;
    
    % Position drop-down
    dd_measurement.Layout.Row = 1;
    dd_measurement.Layout.Column = 2;
    
    % Position axes
    ax.Layout.Row = 2;
    ax.Layout.Column = [1 2];

    %dd_measurement.items = dataset_obj.tab_measurements.v_pressure(:, 1);
    dd_measurement.Items = cellstr(string(1:length(dataset_obj.measurements)));
    dd_measurement.Value = "1";

    function on_change(src, event, ax)
        cla(ax)
        measurement = dataset_obj.measurements(sscanf(event.Value, "%d"));
        measurement.plot_measurement(ax, false);
        lbl_pressure.Text = measurement.v_pressure(1);
    end

    dd_measurement.ValueChangedFcn = {@on_change, ax};

    on_change(0, struct("Value", "1"), ax);

end

