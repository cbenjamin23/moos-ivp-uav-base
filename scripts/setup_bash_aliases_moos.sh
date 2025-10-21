
## Config aliases

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF -a'
alias la='ls -A'
alias l='ls -CF'


# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'



##############################################
#                                            #
#          CUSTOM BASH ALIASES               #
#                                            #
##############################################


## Convinience aliases

alias codebashrc='code ~/.bashrc'
alias codealias='code ~/.bash_aliases'

alias sourcebashrc='source ~/.bashrc'


###################### MOOS - START #########################################


#-------------------------------------------------------
# Misc Aliases
#-------------------------------------------------------
alias pingoe='ping oceanai.mit.edu'


#-------------------------------------------------------
# Suggestions for useful aliases
#-------------------------------------------------------
alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'
alias ls='ls --color=auto'
alias ll='ls -hlG'
alias cdd='cd ..'
alias cddd='cd ../..'
alias emacs='emacs --no-splash '
alias nem='emacs -nw --no-splash '

alias cll='./clean.sh'
alias lll='ktm; cll; ./launch.sh'


## Launching MOOS-IvP

# use MAVSDK_MESSAGE_HANDLER_DEBUGGING=1 for debugging on incoming messages from ardupilot
alias launchSkywalker='cd ~/moos-ivp-uav-base/missions/UAV_Fly;
                       MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 pArduBridge targ_skywalker.moos;'

alias launchSkyfollower='cdmis;
                       MAVSDK_CALLBACK_DEBUGGING=1 MAVSDK_COMMAND_DEBUGGING=1 MAVSDK_PARAMETER_DEBUGGING=1 pArduBridge targ_skyfollower.moos;'


#-------------------------------------------------------
# Some suggestions for MOOS-IvP aliases
#-------------------------------------------------------
# for moos-ivp
alias cdm='cd ~/moos-ivp'
alias cdmm='cdm; cd ivp/missions'
alias cdmma='cdmm; cd s1_alpha'
alias cdmsrc='cdm; cd ivp/src'

# for moos-ivp-uav-base
alias cduav='cd ~/moos-ivp-uav-base/'

alias cdsc='cduav; cd ./scripts'
alias cdmi='cduav; cd ./missions'
alias cdmis='cdmi; cd ./UAV_Fly'
alias bld='cduav; ./build.sh; cd -'

# for moos-ivp-swarm
alias cdsw='cd ~/moos-ivp-swarm/'
alias cdswm='cdsw; cd ./missions'
alias cdswsrc='cdsw; cd ./src'

# for last opened shoreside moos log directoty 
cdlastOpened(){
    cdmis 
    local log=$(cat ".LastOpenedMOOSLogDirectory")
    ## remove .LastOpenedMOOSLogDirectory= from the beginning of log
    cd ${log:29}
}

# building for minirobot
alias build_all_m='cdm; git pull; ./build.sh -m; cd -
                   cdsw; svn up; ./build.sh -m; cd -
                   cduav; ./build.sh; cd -'; 



#-------------------------------------------------------
# Aliases needed at GCS
#-------------------------------------------------------
# The Odroid 1 computer
alias pingo1='ping 10.0.60.110'
alias ssho1='ssh odroid@10.0.60.110'

# The Odroid 2 computer
alias pingo2='ping 10.0.60.115'
alias ssho2='ssh odroid@10.0.60.115'

# Wifi forwarding
alias ip_fw_o='sudo /usr/local/bin/setup_nat_ip_forwarding.sh'

###################### MOOS - END #########################################






###################### MAVSDK - START #########################################


alias mavsdk_configure_build_install="cd ~/moos-ivp-uav-base/MAVSDK;
                                        cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Debug -Bbuild/default -H. -DBUILD_SHARED_LIBS=ON -DSUPERBUILD=ON -DBUILD_TESTS=OFF; 
                                        sudo cmake --build build/default --target install;
                                        cd -"
alias mavsdk_build_install="cd ~/moos-ivp-uav-base/MAVSDK;
                            sudo cmake --build build/default --target install;
                            cd -"   

alias bldm='sudo .;
            mavsdk_build_install;
            bld; 
            cd ~/moos-ivp-uav-base/scripts;
            ./merge_compile_commands.sh;
            cd -'

###################### MAVSDK - END #########################################



####################### PATHS - START #########################################

#-------------------------------------------------------
# Set the Shell Path
#-------------------------------------------------------
PATH=$PATH:/bin
PATH=$PATH:/snap/bin
PATH=$PATH:/usr/bin
PATH=$PATH:/usr/local/bin
PATH=$PATH:/opt/homebrew/bin
PATH=$PATH:/opt/local/bin
PATH=$PATH:/usr/X11/bin
PATH=$PATH:/sbin
PATH=$PATH:/usr/sbin
PATH=$PATH:~/moos-ivp
PATH=$PATH:~/moos-ivp/bin
PATH=$PATH:~/moos-ivp/scripts
PATH=$PATH:~/moos-ivp-uav-base/bin 
PATH=$PATH:~/moos-ivp-swarm/bin 

export PATH


#-------------------------------------------------------
# IvP Helm Behavior directories
#-------------------------------------------------------
if [[ ! -z $IVP_BEHAVIOR_DIRS ]]; then
    export IVP_BEHAVIOR_DIRS=$IVP_BEHAVIOR_DIRS:~/moos-ivp/lib
else
    export IVP_BEHAVIOR_DIRS=~/moos-ivp/lib
fi
IVP_BEHAVIOR_DIRS=$IVP_BEHAVIOR_DIRS:~/moos-ivp-uav-base/lib
IVP_BEHAVIOR_DIRS=$IVP_BEHAVIOR_DIRS:~/moos-ivp-swarm/lib
export IVP_BEHAVIOR_DIRS



IVP_IMAGE_DIRS=~/moos-ivp-uav-base/Maps/NTNU_UAV_Airport
export IVP_IMAGE_DIRS


source ~/moos-ivp-uav-base/scripts/alog_extract_telemetry_csv.sh

####################### PATHS - END #########################################