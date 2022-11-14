const
    axios = require('axios'),
    querystring = require('querystring')

var config;

module.exports.fetchAccessToken = async () => {

    const
        buildingId = config.DEFAULT_BUILDING_ID,
        groupId = config.DEFAULT_BUILDING_GROUP_ID;

    const requestData = querystring.stringify({
        grant_type: 'client_credentials',
        scope: 'robotcall/group:' + buildingId + ":" + groupId,
    })
    const requestConfig = {
        auth: {
            username: config.CLIENT_ID,
            password: config.CLIENT_SECRET,
        },
        headers: {
            'Content-Type': 'application/x-www-form-urlencoded',
        },
    }
    
    try {
        const requestResult = await axios.post(
            config.KONE_AUTH_TOKEN_URL, requestData, requestConfig);
        return requestResult.data.access_token
    } catch (error) {

        error = error || {
            response: {}
        };
        console.error('Error during Access Token fetch', error);
    }
}

module.exports.init = (configData) => {
    config = configData;
}
