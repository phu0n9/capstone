const router = require('express').Router()
const User = require('../model/user.model')

// // create a user a new user
// var testUser = new User({
//     username: 'giang',
//     password: 'giang'
// });
     
// // save the user to database
// testUser.save(function(err) {
//     if (err) throw err;
// })


router.route('/').post(async (req, res) => {
    const checkUser = await User.findOne({username:req.body.username})
    if(checkUser == null){
        return res.status(400).send('Cannot find user')
    }
    try {
        await User.findOne({ username: req.body.username }, function(err, user) {
            if (err) console.log(err)
            try{
                // test a matching password
                user.comparePassword(req.body.password, function(err, isMatch) {
                    if (err) console.log(err)
                    if(isMatch){
                        res.send(user._id)
                    }
                    else {
                        res.send('Incorrect password')
                      }
                })
            }
            catch(e){
                if(e instanceof TypeError){
                    res.status(400).send('Cannot find user')
                }else{
                    res.status(500).send()
                }
            }
        })
    } catch {
      res.status(500).send()
    }
  })


module.exports = router;