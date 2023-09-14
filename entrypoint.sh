#!/bin/bash

# setup dds router environment
source "/dds_router/install/setup.bash"

# Check the value of USE_HUSARNET
if [[ $USE_HUSARNET == "FALSE" ]]; then
    echo "using without Husarnet"
    exec "$@"
else
    for i in {1..7}; do
        husarnet_api_response=$(curl -s http://127.0.0.1:16216/api/status)

        # Check the exit status of curl. If it's 0, the command was successful.
        if [[ $? -eq 0 ]]; then
            if [ "$(echo $husarnet_api_response | yq -r .result.is_ready)" != "true" ]; then
                if [[ $i -eq 7 ]]; then
                    echo "Husarnet API is not ready. Exiting."
                    exit 1
                fi
                echo "Husarnet API is not ready"
                sleep 2
            else
                echo "Husarnet API is ready!"
                break
            fi
        else
            if [[ $i -eq 5 ]]; then
                echo "Failed to connect after 5 retries. Exiting."
                exit 1
            else
                echo "Failed to connect to Husarnet API endpoint. Retrying in 2 seconds..."
                sleep 2
            fi
        fi
    done

    if [ -z "${ROS_DOMAIN_ID}" ]; then
        export ROS_DOMAIN_ID=0
    fi

    case $DISCOVERY in
    SERVER)
        yq '.participants[1].listening-addresses[0].domain = strenv(DS_HOSTNAME)' config.server.template.yaml >DDS_ROUTER_CONFIGURATION_base.yaml
        yq -i '.participants[1].discovery-server-guid.id = env(DS_SERVER_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
        ;;
    CLIENT)
        yq '.participants[1].connection-addresses[0].addresses[0].domain = strenv(DS_HOSTNAME)' config.client.template.yaml >DDS_ROUTER_CONFIGURATION_base.yaml
        yq -i '.participants[1].discovery-server-guid.id = env(DS_CLIENT_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
        yq -i '.participants[1].connection-addresses[0].discovery-server-guid.id = env(DS_SERVER_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
        ;;
    WAN)
        cp config.wan.template.yaml DDS_ROUTER_CONFIGURATION_base.yaml

        export LOCAL_IP=$(echo $husarnet_api_response | yq .result.local_ip)
        yq -i '.participants[1].listening-addresses[0].ip = strenv(LOCAL_IP)' DDS_ROUTER_CONFIGURATION_base.yaml
        yq -i '.participants[1].connection-addresses[0].ip = strenv(LOCAL_IP)' DDS_ROUTER_CONFIGURATION_base.yaml
        ;;
    *)
        echo "Unknown DISCOVERY type"
        exit 1
        ;;
    esac

    yq -i '.participants[0].domain = env(ROS_DOMAIN_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
    yq -i '.participants[0].transport = env(LOCAL_TRANSPORT)' DDS_ROUTER_CONFIGURATION_base.yaml

    rm -f config.yaml.tmp
    rm -f /tmp/loop_done_semaphore

    nohup ./config_daemon.sh &>config_daemon_logs.txt &

    # wait for the semaphore indicating the loop has completed once
    while [ ! -f /tmp/loop_done_semaphore ]; do
        sleep 0.1 # short sleep to avoid hammering the filesystem
    done

    exec "$@"
fi
