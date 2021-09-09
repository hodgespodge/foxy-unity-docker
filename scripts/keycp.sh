KEY_DESTINATION=/.local/share/unity3d/Unity/Unity_lic.ulf
KEY_SOURCE=/shared/Unity_lic.ulf

if [[ ! -f "$KEY_DESTINATION" ]] 
then

    if [[ -f "$KEY_SOURCE" ]]
    then
        echo "copying key"
        mkdir -p $KEY_DESTINATION
        cp $KEY_SOURCE $KEY_DESTINATION
    else
        echo "missing Unity_lic.ulf authentication file"
        echo ""
        echo "Run unityhub to create authentication key: "
        echo "/unityhub --no-sandbox >/dev/null"
        echo ""
        echo "Then copy the key to the shared directory:"
        echo "cp $KEY_DESTINATION $KEY_SOURCE"
        echo ""
        echo "Or copy your Unity_lic.ulf to the shared directory and restart the container"
    fi
fi