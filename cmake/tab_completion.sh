#. ~/.bashrc // why source not usable?
while read line
do
  if [[ $line == "GSLAM_COMPLETION_ENABLED=ON" ]];then
    GSLAM_COMPLETION_ENABLED=ON
  fi
done < ~/.bashrc


if [ -z "$GSLAM_COMPLETION_ENABLED" ];then
echo '
# GSLAM tab completion support
function_gslam_complete()
{
    COMPREPLY=()
    local cur=${COMP_WORDS[COMP_CWORD]};
    local com=${COMP_WORDS[COMP_CWORD-1]};
    local can=$(${COMP_WORDS[*]:0:COMP_CWORD} -help -complete_function_request)
    local reg="-.+"
    if [[ $com =~ $reg ]];then
      COMPREPLY=($(compgen -f -W "$can" -- $cur))
    else
      COMPREPLY=($(compgen -W "$can" -- $cur))
    fi
}

complete -F function_gslam_complete "gslam"
GSLAM_COMPLETION_ENABLED=ON
'>> ~/.bashrc
else
  echo "GSLAM tab completion has already supported."
fi


