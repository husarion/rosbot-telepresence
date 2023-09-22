#!/bin/bash

create_config_husarnet() {
    if [ -z "${ROS_DOMAIN_ID}" ]; then
        export ROS_DOMAIN_ID=0
    fi

    # Check if DISCOVERY_SERVER_PORT environment variable exists
    if [ -z "${DISCOVERY_SERVER_PORT}" ]; then
        # DISCOVERY_SERVER_PORT is not set.

        # Check if ROS_DISCOVERY_SERVER environment variable exists
        if [ -z "${ROS_DISCOVERY_SERVER}" ]; then
            echo "Launching Initial Peers config"

            cp config.wan.template.yaml DDS_ROUTER_CONFIGURATION_base.yaml

            export LOCAL_IP=$(echo $husarnet_api_response | yq .result.local_ip)
            yq -i '.participants[1].listening-addresses[0].ip = strenv(LOCAL_IP)' DDS_ROUTER_CONFIGURATION_base.yaml
            yq -i '.participants[1].connection-addresses[0].ip = strenv(LOCAL_IP)' DDS_ROUTER_CONFIGURATION_base.yaml
        else
            echo "Launching ROS Discovery Server - Client config"

            # Splitting the string into an array using space, comma, or semicolon as the delimiter
            IFS=' ,;' read -ra DS_LIST <<< "$ROS_DISCOVERY_SERVER"
            
            # Regular expression to match hostname:port or [ipv6addr]:port
            DS_REGEX="^(([a-zA-Z0-9-]+):([0-9]+)|\[(([a-fA-F0-9]{1,4}:){1,7}[a-fA-F0-9]{1,4}|[a-fA-F0-9]{0,4})\]:([0-9]+))$"

            cp config.client.template.yaml DDS_ROUTER_CONFIGURATION_base.yaml

            yq -i '.participants[1].discovery-server-guid.id = env(CLIENT_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
            yq -i '.participants[1].connection-addresses = []' DDS_ROUTER_CONFIGURATION_base.yaml

            echo "Connecting to:"
            # Loop over Discovery Servers
            for ds in ${DS_LIST[@]}; do
                if [[ "${ds}" =~ $DS_REGEX ]]; then
                    # Extract HOST and PORT from the match results
                    if [ -z "${BASH_REMATCH[4]}" ]; then
                        # If it's in hostname:port format
                        HOST="${BASH_REMATCH[2]}"
                        export PORT="${BASH_REMATCH[3]}"

                        ipv6=$(echo $husarnet_api_response | yq .result.host_table | yq -r ".$HOST")

                        if [[ "$ipv6" == "null" || -z "$ipv6" ]]; then
                            echo "Error: IPv6 address not found for $HOST"
                            exit 1
                        else
                            export HOST=$ipv6
                        fi
                    else
                        # If it's in [ipv6addr]:port format
                        HOST="${BASH_REMATCH[4]}"
                        export PORT="${BASH_REMATCH[6]}"

                        # Extract all IP addresses from the host_table
                        IP_ADDRESSES=$(echo $husarnet_api_response | yq '.result.host_table[]')

                        # Iterate over each address in IP_ADDRESSES
                        address_found=false
                        IFS=$'\n' # Set Internal Field Separator to newline for the loop
                        for address in $IP_ADDRESSES; do
                            if [[ "$address" == "$HOST" ]]; then
                                address_found=true
                                break
                            fi
                        done

                        if $address_found; then
                            export HOST
                        else
                            echo "Error: $HOST address not found"
                            exit 1
                        fi
                    fi

                    if [[ ! ($PORT -le 65535) ]]; then
                        echo "Discovery Server Port is not a valid number or is outside the valid range (0-65535)."
                        exit 1
                    fi

                    yq -i '.participants[1].connection-addresses += 
                            { 
                                "discovery-server-guid": 
                                { 
                                    "ros-discovery-server": true, 
                                    "id": env(SERVER_ID) 
                                }, 
                                "addresses": 
                                [ 
                                    { 
                                        "ip": strenv(HOST), 
                                        "port": env(PORT), 
                                        "transport": "udp" 
                                    } 
                                ] 
                            }' DDS_ROUTER_CONFIGURATION_base.yaml

                    echo "[$HOST]:$PORT (server id: $SERVER_ID)"
                else
                    echo "ROS_DISCOVERY_SERVER does not have a valid format: $ds"
                    exit 1
                fi
            done
        fi

    else
        echo "Launching ROS Discovery Server - Server config"
        # Check if the value is a number and smaller than 65535
        if [[ "$DISCOVERY_SERVER_PORT" =~ ^[0-9]+$ && $DISCOVERY_SERVER_PORT -lt 65535 ]]; then
            # DISCOVERY_SERVER_PORT is set and its value is smaller than 65535.

            export LOCAL_IP=$(echo $husarnet_api_response | yq .result.local_ip)

            echo "On different hosts, set the ROS_DISCOVERY_SERVER=[$LOCAL_IP]:$DISCOVERY_SERVER_PORT"

            yq '.participants[1].listening-addresses[0].ip = strenv(LOCAL_IP)' config.server.template.yaml >DDS_ROUTER_CONFIGURATION_base.yaml
            yq -i '.participants[1].listening-addresses[0].port = env(DISCOVERY_SERVER_PORT)' DDS_ROUTER_CONFIGURATION_base.yaml
            yq -i '.participants[1].discovery-server-guid.id = env(SERVER_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
        else
            echo "DISCOVERY_SERVER_PORT value is not a valid number or is greater than or equal to 65535."
            # Insert other commands here if needed
            exit 1
        fi
    fi
}

create_config_local() {
    cp config.local.template.yaml DDS_ROUTER_CONFIGURATION_base.yaml
    yq -i '.participants[1].domain = env(ROS_DOMAIN_ID_2)' DDS_ROUTER_CONFIGURATION_base.yaml
}

if [[ $AUTO_CONFIG == "TRUE" ]]; then

    # Check the value of USE_HUSARNET
    if [[ $USE_HUSARNET == "FALSE" ]]; then
        echo "Using LAN setup."
        create_config_local
        export husarnet_ready=false
    else
        for i in {1..7}; do
            husarnet_api_response=$(curl -s http://127.0.0.1:16216/api/status)

            # Check the exit status of curl. If it's 0, the command was successful.
            if [[ $? -eq 0 ]]; then
                if [ "$(echo $husarnet_api_response | yq -r .result.is_ready)" != "true" ]; then
                    if [[ $i -eq 7 ]]; then
                        echo "Husarnet API is not ready."
                        if [[ $EXIT_IF_HUSARNET_NOT_AVAILABLE == "TRUE" ]]; then
                            echo "Exiting."
                            exit 1
                        else
                            echo "Using LAN setup."
                            create_config_local
                            export husarnet_ready=false
                            break
                        fi
                    else
                        echo "Husarnet API is not ready"
                        sleep 2
                    fi
                else
                    echo "Husarnet API is ready!"
                    create_config_husarnet
                    export husarnet_ready=true
                    break
                fi
            else
                if [[ $i -eq 5 ]]; then
                    echo "Can't reach Husarnet Daemon HTTP API after 5 retries"
                    if [[ $EXIT_IF_HUSARNET_NOT_AVAILABLE == "TRUE" ]]; then
                        echo "Exiting."
                        exit 1
                    else
                        echo "Using LAN setup."
                        create_config_local
                        export husarnet_ready=false
                        break
                    fi
                else
                    echo "Failed to connect to Husarnet API endpoint. Retrying in 2 seconds..."
                    sleep 2
                fi
            fi
        done

    fi

    if [[ -n $WHITELIST_INTERFACES ]]; then
        # Splitting the string into an array using space, comma, or semicolon as the delimiter
        IFS=' ,;' read -ra IP_LIST <<< "$WHITELIST_INTERFACES"

        # IP address validation regex pattern
        IP_REGEX="^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$"

        yq -i '.participants[0].whitelist-interfaces = []' DDS_ROUTER_CONFIGURATION_base.yaml
        # Loop over the IP addresses
        for ip in ${IP_LIST[@]}; do
            if [[ "${ip}" =~ $IP_REGEX ]]; then
                export ip
                yq -i '.participants[0].whitelist-interfaces += env(ip)' DDS_ROUTER_CONFIGURATION_base.yaml
            else
                echo "WHITELIST_INTERFACES: $ip is NOT a valid IP address"
            fi
        done

        if [[ $(yq '.participants[0].whitelist-interfaces' DDS_ROUTER_CONFIGURATION_base.yaml) == "[]" ]]; then
            yq -i 'del(.participants[0].whitelist-interfaces)' DDS_ROUTER_CONFIGURATION_base.yaml
        fi
    fi

    yq -i '.participants[0].domain = env(ROS_DOMAIN_ID)' DDS_ROUTER_CONFIGURATION_base.yaml
    yq -i '.participants[0].transport = env(LOCAL_TRANSPORT)' DDS_ROUTER_CONFIGURATION_base.yaml

    rm -f config.yaml.tmp
    rm -f /tmp/loop_done_semaphore

    # Start a config_daemon
    rm -f config_daemon_logs_pipe
    mkfifo config_daemon_logs_pipe
    cat <config_daemon_logs_pipe &
    pkill -f config_daemon.sh
    nohup ./config_daemon.sh >config_daemon_logs_pipe 2>&1 &
    # nohup ./config_daemon.sh &>config_daemon_logs.txt &

    # wait for the semaphore indicating the loop has completed once
    while [ ! -f /tmp/loop_done_semaphore ]; do
        sleep 0.1 # short sleep to avoid hammering the filesystem
    done

fi

# setup dds router environment
source "/dds_router/install/setup.bash"

exec "$@"
