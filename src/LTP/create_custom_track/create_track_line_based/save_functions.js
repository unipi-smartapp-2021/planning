//save yellow and blue cones as a JSON file
function sv_json() {
    //creating final result
    const res = { yellow_cones: null, blue_cones: null };
    res.yellow_cones = JSON.parse(JSON.stringify(yellow_cones))
    res.yellow_cones.map(pt => { pt_tmp = to_custom_coordinate_system(pt); pt.x = pt_tmp.x; pt.y = pt_tmp.y; return pt });
    res.blue_cones = JSON.parse(JSON.stringify(blue_cones))
    res.blue_cones.map(pt => { pt_tmp = to_custom_coordinate_system(pt); pt.x = pt_tmp.x; pt.y = pt_tmp.y; return pt });

    const doc = document.createElement("a");
    const file = new Blob([JSON.stringify(res, null, '\t')], { type: 'text/plain' });
    doc.href = URL.createObjectURL(file);
    doc.download = track_name_input.value() + '.json';
    doc.click();
}

//save yellow and blue cones as a YAML file
function sv_yaml() {
    //creating final result
    const res = { yellow_cones: null, blue_cones: null };
    res.yellow_cones = JSON.parse(JSON.stringify(yellow_cones))
    res.yellow_cones.map(pt => { pt_tmp = to_custom_coordinate_system(pt); pt.x = pt_tmp.x; pt.y = pt_tmp.y; return pt });
    res.blue_cones = JSON.parse(JSON.stringify(blue_cones))
    res.blue_cones.map(pt => { pt_tmp = to_custom_coordinate_system(pt); pt.x = pt_tmp.x; pt.y = pt_tmp.y; return pt });

    let yaml_res = "cones:\n"
    res.yellow_cones.forEach(cone => {
        yaml_res += "- position:\n\t- " + cone.x + "\n\t- " + cone.y + "\n\t- 0\n\ttype: small yellow\n"
    });
    res.blue_cones.forEach(cone => {
        yaml_res += "- position:\n\t- " + cone.x + "\n\t- " + cone.y + "\n\t- 0\n\ttype: small blue\n"
    });

    yaml_res += "description: track generated with WS-LBT\ninit_offset: TODO\nname: " + track_name_input.value()

    const doc = document.createElement("a");
    const file = new Blob([yaml_res], { type: 'text/plain' });
    doc.href = URL.createObjectURL(file);
    doc.download = track_name_input.value() + '.yaml';
    doc.click();
}