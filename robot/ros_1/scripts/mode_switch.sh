mode=""
module=""

while [ "$1" != "" ]; do
    case $1 in
        -mode | --map-mode )        shift
                                    mode=$1
                                    ;;
        -module | --module-name )   shift
                                    module=$1
                                    ;;
    esac
    shift
done

if [ "$module" = "afs" ]; then

    if [ "$mode" = "M" ]; then
        screen -S sar.amcl -X at "#" stuff ^C
        screen -S sar.micvision -X at "#" stuff ^C
        screen -S sar.map_server -X at "#" stuff ^C
        screen -S sar.move_base -X at "#" stuff ^C
        screen -S sar.pal_depth -X at "#" stuff ^C
        screen -dmS sar.gmapping sh -c "roslaunch afs_navigation gmapping.launch; exec bash"


    elif [ "$mode" = "N" ]; then
        screen -S sar.gmapping -X at "#" stuff ^C
        screen -dmS sar.amcl sh -c "roslaunch afs_navigation amcl.launch; exec bash"
        screen -dmS sar.micvision sh -c "roslaunch afs_navigation micvision.launch; exec bash"
        screen -dmS sar.map_server sh -c "rosrun map_server map_server /home/dev/sar/maps/newmap.yaml; exec bash"
        screen -dmS sar.move_base sh -c "roslaunch afs_navigation move_base.launch; exec bash"
        screen -dmS sar.pal_depth sh -c "rosrun dreamvu_pal_camera afs_person_depth; exec bash"

    fi
        screen -ls
fi
