const router = require('express').Router()
const User = require('../model/user.model')

// // create a user a new user
// var testUser = new User({
//     username: 'hello',
//     password: 'hello123'
// });
     
// // save the user to database
// testUser.save(function(err) {
//     if (err) throw err;
// });

// router.route('/').get((req,res) => {
//     User.find()
//     .then(user => res.json(user))
//     .catch(err => res.status(400).json('Error: '+err));
// })

// router.route('/').post(async (req,res) => {
//     try{
//         const hashedPassword = await bcrypt.hash(req.body.password,10)
//         const user = {
//             username: req.body.username, 
//             password: hashedPassword
//         }
//         res.status(200).send(user)
//     }
//     catch{
//         res.status(500).send("posting")
//     }
// })

// router.route('/username').get((req,res) => {
//     User.findOne({ username: req.body.username})
//     .then((user) => res.json(user))
//     .catch((err) =>res.status(400).json('Error '+err))
// })

// router.route('/username').post((req,res) => {
//     User.findOne({ username: req.body.username})
//     .then((user) => console.log(user._id))
//     .catch((err) =>res.status(400).json('Error '+err))
// })

router.route('/').post(async (req, res) => {
    const checkUser = await User.findOne({username:req.body.username})
    if(checkUser == null){
        return res.status(400).send('Cannot find user')
    }
    try {
        await User.findOne({ username: req.body.username }, function(err, user) {
            if (err) throw err
            try{
                // test a matching password
                user.comparePassword(req.body.password, function(err, isMatch) {
                    if (err) throw err
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