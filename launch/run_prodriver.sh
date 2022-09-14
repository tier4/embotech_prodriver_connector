
prodriver_path=$1
run_gui=$2

if [ "$run_gui" == "true" ]; then
    cd $prodriver_path/tools/
    ./developer-ui &
    dev_ui_pid=$!
    trap "kill $dev_ui_pid" SIGINT SIGTERM EXIT
fi

cd $prodriver_path
# give some times to the prodriver connector to setup the network
sleep 2 && ./run_prodriver.sh