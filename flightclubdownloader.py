import requests

def get_trajectory(idS):
    api ="https://api.flightclub.io/v2/simulation?X-Api-Key=insertkey&launchLibraryV2Id="
    r = requests.get(api+str(idS))
    json = r.json()
    s=";"
    with open("json.csv", 'w') as f:
        f.write(str(json))
    #for event in json["eventLog"]:
     #   print(event["key"]+":  "+event["value"])
    i =0
    ilist = []
    for stage in json[0]['data']['stageTrajectories']:
        i=i+1
        ilist.append(i)
        with open('stage-'+str(i)+".csv", 'w') as f:
            out=""
            out+=str('time')+","
            out+=str('altitude')+","
            out+=str('latitude')+","
            out+=str('longitude')+","
            f.write(out+"\n")
            for d in stage["telemetry"]:
                out=""
                out+=str(d['time'])+","
                out+=str(d['altitude'])+","
                out+=str(d['latitude'])+","
                out+=str(d['longitude'])+","
                f.write(out+"\n")
    return(ilist)
