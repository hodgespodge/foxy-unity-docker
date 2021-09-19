EDITOR=/opt/unity/editors/2020.3.11f1/Editor/Unity
LICENSE_SOURCE=/shared/*.ulf
LICENSE=false

for file in $LICENSE_SOURCE
do  
    [ -e "$file" ] || continue
    echo "License file $file found!"
    $EDITOR -batchmode -manualLicenseFile $f -logfile
    LICENSE=true
    break
done


if [ ! $LICENSE ]
then
    echo "#################################################################"
    echo ".ulf file not found in shared folder. Creating new license activation file."
    $EDITOR -batchmode -createManualActivationFile -logfile
    mv Unity_v2020.3.11f1.alf /shared
    echo ""
    echo "Unity_v2020.3.11f1.alf added to shared folded."
    echo "Go to https://license.unity3d.com/manual and get a .ulf license. Add that license to shared."
    echo "Then run ' exec bash '"
    "#################################################################"
fi

